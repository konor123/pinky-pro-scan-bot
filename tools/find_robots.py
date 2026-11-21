#!/usr/bin/env python3
"""로봇 AP 이름(SSID, 로봇 LCD에 나옴)으로 같은 Wi-Fi에서 로봇 주소를 찾는다. 표준 라이브러리만 쓴다(Linux·macOS).

로봇 AP 이름은 pinky_<eth0 MAC 끝 4자리>이고(로봇 ~/ap/ap_setup.py), Wi-Fi(wlan0) MAC은 eth0 MAC + 1이다.
지난 주소가 SSH 포트로 응답하고 MAC이 맞으면 그대로 쓰고, 아니면 이 컴퓨터의 망 주소마다 UDP를 한 번씩 보내
ARP 표를 채운 뒤 MAC 끝자리가 맞는 주소를 고른다.
Docker 안에서는 바깥 Wi-Fi의 ARP가 보이지 않으니 호스트에서 실행한다.

  tools/find_robots.py pinky_d52c pinky_6437   SSID마다 주소 출력
  tools/find_robots.py --env .env              PINKY1_SSID·PINKY2_SSID로 찾아 PINKY1_HOST·PINKY2_HOST를 고쳐 쓴다
                                               (scripts/ssh_aliases.sh가 부른다)
"""
import argparse
import ipaddress
import re
import socket
import subprocess
import sys
import time
from pathlib import Path

MAX_HOSTS = 4094  # /20보다 큰 망은 훑지 않는다
SKIP_IF = ("lo", "tun", "docker", "br-", "veth", "virbr", "utun", "awdl", "llw", "bridge")
IPV4 = re.compile(r"\d+\.\d+\.\d+\.\d+")


def ssid_suffixes(ssid):
    """SSID → 로봇 MAC 끝 2바이트 후보(eth0, wlan0 = eth0 + 1)."""
    m = re.fullmatch(r"pinky_([0-9a-fA-F]{4})", ssid.strip())
    if not m:
        raise ValueError(f"SSID가 pinky_XXXX 꼴이 아님: {ssid}")
    n = int(m.group(1), 16)
    return {n, (n + 1) & 0xFFFF}


def norm_mac(mac):
    parts = mac.lower().replace("-", ":").split(":")
    if len(parts) != 6:
        return None
    try:
        return ":".join(f"{int(p, 16):02x}" for p in parts)
    except ValueError:
        return None


def parse_neigh(text):
    """`ip neigh`(Linux)·`arp -an`(macOS) 출력 → [(ip, mac, 상태)]. 상태는 Linux만 있다(REACHABLE 등)."""
    out = []
    for line in text.splitlines():
        m = re.match(r"(\d+\.\d+\.\d+\.\d+) .*lladdr (\S+)\s*(.*)", line)
        if m:
            state = m.group(3).split()[-1] if m.group(3).split() else ""
            mac = norm_mac(m.group(2))
            if mac and state not in ("FAILED", "INCOMPLETE"):
                out.append((m.group(1), mac, state))
            continue
        m = re.match(r"\S+ \((\d+\.\d+\.\d+\.\d+)\) at (\S+)", line)
        if m and norm_mac(m.group(2)):
            out.append((m.group(1), norm_mac(m.group(2)), ""))
    return out


def parse_addrs(text):
    """`ip -o -4 addr`(Linux)·`ifconfig`(macOS) 출력 → [IPv4Interface]. 루프백·VPN·도커 인터페이스는 뺀다."""
    nets, iface = [], ""
    for line in text.splitlines():
        m = re.match(r"\d+:\s+(\S+)\s+inet (\d+\.\d+\.\d+\.\d+/\d+)", line)
        if m:
            if not m.group(1).startswith(SKIP_IF):
                nets.append(ipaddress.ip_interface(m.group(2)))
            continue
        m = re.match(r"(\S+): flags=", line)
        if m:
            iface = m.group(1)
            continue
        m = re.match(r"\s+inet (\d+\.\d+\.\d+\.\d+) netmask 0x([0-9a-fA-F]{8})", line)
        if m and not iface.startswith(SKIP_IF):
            prefix = bin(int(m.group(2), 16)).count("1")
            nets.append(ipaddress.ip_interface(f"{m.group(1)}/{prefix}"))
    return [n for n in nets if not (n.ip.is_loopback or n.ip.is_link_local)]


def pick(ssid, table):
    """ARP 표에서 SSID에 맞는 주소. 여러 개면 REACHABLE인 것만 남긴다. → [ip]"""
    want = ssid_suffixes(ssid)
    hits = [(ip, st) for ip, mac, st in table if int(mac.replace(":", "")[-4:], 16) in want]
    live = [ip for ip, st in hits if st == "REACHABLE"]
    return live if len(hits) > 1 and live else [ip for ip, _ in hits]


def _run(cmd):
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=10).stdout
    except (OSError, subprocess.SubprocessError):
        return ""


def local_nets():
    return parse_addrs(_run(["ip", "-o", "-4", "addr", "show"]) or _run(["ifconfig"]))


def neighbours():
    return parse_neigh(_run(["ip", "neigh", "show"]) or _run(["arp", "-an"]))


def alive(ip, port=22, timeout=1.0):
    """지난 주소가 지금 응답하는지(SSH 포트 TCP 연결). ARP 표의 오래된 기록만 보고 고르지 않게 한다."""
    try:
        with socket.create_connection((ip, port), timeout=timeout):
            return True
    except OSError:
        return False


def poke(ips, wait):
    """주소마다 UDP(포트 9) 한 번. 커널이 ARP로 MAC을 묻는다."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        for i, ip in enumerate(ips):
            try:
                s.sendto(b"", (str(ip), 9))
            except OSError:
                pass
            if i % 64 == 63:
                time.sleep(0.2)
    time.sleep(wait)


def find(ssids, hints=(), nets=None):
    """SSID → 주소 dict. 먼저 지금 응답하는 hints(지난번 주소)만 보고, 없으면 망 전체를 훑는다. 못 찾으면 빠진다."""
    found, todo = {}, list(ssids)
    hints = [h for h in hints if h and IPV4.fullmatch(h) and alive(h)]
    if hints:
        table = [e for e in neighbours() if e[0] in hints]
        for s in list(todo):
            ips = pick(s, table)
            if len(ips) == 1:
                found[s] = ips[0]
                todo.remove(s)
    if todo:
        hosts = []
        for n in nets if nets is not None else local_nets():
            if n.network.num_addresses - 2 > MAX_HOSTS:
                print(f"{n.network}: 너무 커서 훑지 않음(--subnet으로 좁힌다)", file=sys.stderr)
                continue
            hosts += list(n.network.hosts())
        poke(hosts, 3.0)
        table = neighbours()
        for s in todo:
            ips = pick(s, table)
            if len(ips) == 1:
                found[s] = ips[0]
            elif ips:
                print(f"{s}: 주소가 여러 개 {ips}", file=sys.stderr)
    return found


def read_env(path):
    env = {}
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if line and not line.startswith("#") and "=" in line:
            k, _, v = line.partition("=")
            env[k.strip()] = v.strip().strip('"').strip("'")
    return env


def update_env(path, values):
    """.env의 KEY=값 줄을 바꾼다(없으면 끝에 붙임). 다른 줄과 주석은 그대로."""
    path = Path(path)
    lines = path.read_text(encoding="utf-8").splitlines()
    for key, val in values.items():
        for i, line in enumerate(lines):
            if re.match(rf"\s*{re.escape(key)}\s*=", line):
                lines[i] = f"{key}={val}"
                break
        else:
            lines.append(f"{key}={val}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main(argv=None):
    ap = argparse.ArgumentParser(description="SSID로 로봇 주소 찾기")
    ap.add_argument("ssid", nargs="*", help="pinky_XXXX")
    ap.add_argument("--env", help="PINKY1_SSID·PINKY2_SSID를 읽고 PINKY1_HOST·PINKY2_HOST를 고쳐 쓸 .env")
    ap.add_argument("--subnet", action="append", help="훑을 망(CIDR). 기본은 이 컴퓨터의 IPv4 망")
    args = ap.parse_args(argv)

    names, hints = {}, []
    if args.env:
        env = read_env(args.env)
        for i in (1, 2):
            if env.get(f"PINKY{i}_SSID"):
                names[env[f"PINKY{i}_SSID"]] = f"PINKY{i}_HOST"
                hints.append(env.get(f"PINKY{i}_HOST", ""))
    names.update({s: None for s in args.ssid})
    if not names:
        ap.error("SSID가 없다(인자나 .env의 PINKY1_SSID·PINKY2_SSID)")

    nets = [ipaddress.ip_interface(s) for s in args.subnet] if args.subnet else None
    found = find(list(names), hints, nets)
    for ssid, key in names.items():
        label = f"{key.split('_')[0].lower()} " if key else ""
        print(f"{label}{ssid} → {found[ssid]}" if ssid in found else
              f"{label}{ssid}: 못 찾음(로봇이 켜져 있고 이 컴퓨터와 같은 Wi-Fi인지 본다)")
    if args.env:
        update_env(args.env, {key: found[s] for s, key in names.items() if key and s in found})
    return 0 if len(found) == len(names) else 1


if __name__ == "__main__":
    sys.exit(main())

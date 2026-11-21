"""tools/find_robots.py: SSID → MAC 끝자리, ARP·주소 출력 해석, 지난 주소 확인, .env 고쳐 쓰기. 주소·MAC은 지어낸 값."""
import ipaddress

import pytest

from tools import find_robots as fr

LINUX_NEIGH = """10.20.0.1 dev wlan0 lladdr 02:aa:bb:cc:dd:01 REACHABLE
10.20.0.74 dev wlan0 lladdr 02:11:22:48:d5:2d REACHABLE
10.20.0.65 dev wlan0 lladdr 02:33:44:07:64:38 STALE
10.20.0.70 dev wlan0 lladdr 02:33:44:07:64:38 STALE
10.20.0.99 dev wlan0  FAILED
10.20.0.15 dev wlan0 lladdr 02:33:44:07:13:00 REACHABLE
"""
MAC_ARP = """? (10.20.0.1) at 2:aa:bb:cc:dd:1 on en0 ifscope [ethernet]
? (10.20.0.65) at 2:33:44:7:64:38 on en0 ifscope [ethernet]
? (10.20.0.99) at (incomplete) on en0 ifscope [ethernet]
"""
LINUX_ADDR = """1: lo    inet 127.0.0.1/8 scope host lo\\       valid_lft forever preferred_lft forever
3: wlan0    inet 10.20.0.48/22 brd 10.20.3.255 scope global dynamic noprefixroute wlan0
5: tun0    inet 100.64.0.5/32 scope global tun0
6: docker0    inet 172.17.0.1/16 brd 172.17.255.255 scope global docker0
"""
MAC_IFCONFIG = """lo0: flags=8049<UP,LOOPBACK,RUNNING,MULTICAST> mtu 16384
\tinet 127.0.0.1 netmask 0xff000000
en0: flags=8863<UP,BROADCAST,SMART,RUNNING,SIMPLEX,MULTICAST> mtu 1500
\tinet 10.20.0.50 netmask 0xfffffc00 broadcast 10.20.3.255
utun3: flags=8051<UP,POINTOPOINT,RUNNING,MULTICAST> mtu 1280
\tinet 100.64.1.2 netmask 0xffffffff
"""
NETS = [ipaddress.ip_interface("10.20.0.48/22")]


def test_ssid_suffixes():
    assert fr.ssid_suffixes("pinky_d52c") == {0xD52C, 0xD52D}
    assert fr.ssid_suffixes("pinky_ffff") == {0xFFFF, 0x0000}
    with pytest.raises(ValueError):
        fr.ssid_suffixes("pinky_6437x")


def test_parse_neigh_linux_and_macos():
    lin = fr.parse_neigh(LINUX_NEIGH)
    assert ("10.20.0.74", "02:11:22:48:d5:2d", "REACHABLE") in lin
    assert all(ip != "10.20.0.99" for ip, _, _ in lin)
    mac = fr.parse_neigh(MAC_ARP)
    assert ("10.20.0.65", "02:33:44:07:64:38", "") in mac
    assert len(mac) == 2


def test_parse_addrs():
    assert fr.parse_addrs(LINUX_ADDR) == [ipaddress.ip_interface("10.20.0.48/22")]
    assert fr.parse_addrs(MAC_IFCONFIG) == [ipaddress.ip_interface("10.20.0.50/22")]


def test_pick():
    table = fr.parse_neigh(LINUX_NEIGH)
    assert fr.pick("pinky_d52c", table) == ["10.20.0.74"]   # wlan0 = eth0 + 1
    assert fr.pick("pinky_12ff", table) == ["10.20.0.15"]   # 끝자리 넘김(12ff + 1 = 1300)
    assert fr.pick("pinky_6437", table) == ["10.20.0.65", "10.20.0.70"]  # 둘 다 STALE이면 고르지 않음
    live = table + [("10.20.0.71", "02:33:44:07:64:38", "REACHABLE")]
    assert fr.pick("pinky_6437", live) == ["10.20.0.71"]
    assert fr.pick("pinky_4693", table) == []


@pytest.fixture
def lan(monkeypatch):
    """table: 지금 ARP 표, up: SSH 포트가 응답하는 주소, pokes: 훑은 주소 목록."""
    state = {"table": fr.parse_neigh(LINUX_NEIGH), "up": {"10.20.0.74"}, "pokes": []}
    monkeypatch.setattr(fr, "poke", lambda ips, wait: state["pokes"].append(list(ips)))
    monkeypatch.setattr(fr, "neighbours", lambda: state["table"])
    monkeypatch.setattr(fr, "alive", lambda ip, port=22, timeout=1.0: ip in state["up"])
    return state


def test_find_uses_live_hint(lan):
    assert fr.find(["pinky_d52c"], ["10.20.0.74"], NETS) == {"pinky_d52c": "10.20.0.74"}
    assert lan["pokes"] == []                                   # 지난 주소가 응답하고 MAC이 맞으면 훑지 않음
    assert fr.find(["pinky_d52c"], ["10.20.0.80"], NETS) == {"pinky_d52c": "10.20.0.74"}
    assert len(lan["pokes"]) == 1 and len(lan["pokes"][0]) == 1022   # 틀린 지난 주소면 /22 전체


def test_find_ignores_stale_hint(lan):
    """로봇이 .65 → .71로 옮겼는데 ARP 표에 .65가 STALE로 남은 경우."""
    lan["table"] = [e for e in lan["table"] if e[0] != "10.20.0.70"] + [("10.20.0.71", "02:33:44:07:64:38", "REACHABLE")]
    lan["up"] = {"10.20.0.71"}
    assert fr.find(["pinky_6437"], ["10.20.0.65"], NETS) == {"pinky_6437": "10.20.0.71"}


def test_env_round_trip(tmp_path, monkeypatch):
    env = tmp_path / ".env"
    env.write_text("# 주석\nPINKY_USER=pinky\nPINKY1_SSID=pinky_d52c\nPINKY2_SSID=pinky_6437\nPINKY1_HOST=10.9.9.9\n",
                   encoding="utf-8")
    monkeypatch.setattr(fr, "find", lambda ssids, hints, nets: {"pinky_d52c": "10.20.0.74"})
    assert fr.main(["--env", str(env)]) == 1                   # pinky_6437은 못 찾음
    text = env.read_text(encoding="utf-8")
    assert "PINKY1_HOST=10.20.0.74\n" in text and "10.9.9.9" not in text
    assert "PINKY2_HOST" not in text and text.startswith("# 주석\n")

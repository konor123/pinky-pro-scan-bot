"""웹 화면의 상태 기계: 로봇 연결, 미리보기, 스캔, 보강 촬영(docs/specs/webui.md).

한 번에 한 작업(scan, focus)만. 작업 중에는 미리보기를 멈춘다(로봇마다 제어 연결과 카메라는 하나).
취소하면 두 로봇에 abort(제어 연결이 닫힘), 작업이 끝나면 다시 연결한다.
끝 상태(done·failed·cancelled)는 정리(_finish) 뒤에 낸다. 그때까지 busy()가 참이라 새 작업·다시 연결이 끼어들지 못한다.
"""
import json
import math
import subprocess
import threading
import time
import traceback
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .. import dataset, station
from ..config import ORDER, ROOT, load_board
from ..pair.coordinator import PairScan
from ..robot import align, drive, tagpose
from ..robot import link as robot_link
from ..robot.unit import RobotScan, StartError
from . import alarms as alarm_mod
from . import focus, preview
from . import recipe as recipe_mod

RECONNECT_S = 10.0       # 작업이 없을 때 끊긴 로봇에 다시 연결해 보는 주기
STACK_TIMEOUT_S = 90.0   # scripts/robot.sh 한 번(ssh로 켜기·끄기·상태)의 한도
STACK_NAME = "robot_scan"
PREVIEW_STALE_S = 30.0   # 이보다 오래된 미리보기 각도는 출발 조건에 쓰지 않는다
POLL_S = 0.1


class Busy(RuntimeError):
    """다른 작업이 진행 중."""


class NotReady(RuntimeError):
    """시작 조건이 맞지 않음(연결, 각도 차, 스캔 없음, 재구성 중 취소)."""


@dataclass
class Slot:
    role: str
    addr: tuple
    link: object = None
    serial: str = None
    arc: bool = None
    error: str = None
    preview: dict = None
    preview_t: float = None
    jpeg: bytes = b""
    seq: int = 0                    # 미리보기 번호(바뀌면 화면이 사진을 다시 받는다)
    was_connected: bool = False     # 끊기면 알람을 낼지(일부러 끊으면 False)
    obstacle: dict = None           # 주행 중 막혀 기다리는 중이면 {event, view, since}
    stack: str = "unknown"          # 로봇 구동 스택(robot_scan): unknown·running·stopped·starting·stopping·blocked·error
    stack_msg: str = ""
    stack_busy: bool = False
    user_stopped: bool = False      # 화면에서 끈 로봇(다시 켤 때까지 다시 연결하지 않는다)
    fix: tuple = None               # (base_pose, origin_odom): 실시간 판 좌표 추정용 기준
    trail: deque = field(default_factory=lambda: deque(maxlen=600))
    shots: list = field(default_factory=list)
    last_shot_t: float = None
    last_shot_k: int = None
    last_shot_use: str = None
    bypassing: bool = False

    def live_link(self):
        """살아 있는 링크 또는 None. link는 다른 스레드가 바꾸므로 한 번만 읽는다."""
        lk = self.link
        return lk if lk is not None and lk.alive() else None

    def connected(self):
        return self.live_link() is not None


@dataclass
class Job:
    kind: str
    state: str = "running"          # running → done | failed | cancelled(정리를 마친 뒤에 바뀐다)
    message: str = ""
    scan_id: str = None
    result: dict = None
    progress: dict = field(default_factory=dict)
    started: float = field(default_factory=time.monotonic)
    ended: float = None
    pair: object = None
    units: dict = None              # focus 등 pair가 없는 작업의 {role: RobotScan}
    scan_dir: Path = None
    user_cancel: bool = False       # 사용자가 누른 취소(실패와 구분)
    recipe: str = "default"
    cfg: dict = None                # 이 작업이 쓰는 설정(시작할 때 고정)
    thread: threading.Thread = None


class Session:
    def __init__(self, cfg, cfg_path, addrs, clock, data_dir=None, roles=ORDER, fake=False, recipes_path=None,
                 robot_sh=None, fleet=None):
        self.base_cfg, self.cfg_path, self.clock, self.fake = cfg, cfg_path, clock, bool(fake)
        self.fleet = fleet
        self.board_path, self.board = load_board(cfg)
        self.recipes = recipe_mod.load(recipes_path or ROOT / "config" / "recipes.yaml")
        board_json = json.loads(Path(self.board_path).read_text(encoding="utf-8"))
        self.board_json = board_json
        for name, r in self.recipes.items():        # 이 판과 원에서 말이 되는지
            recipe_mod.check(name, recipe_mod.apply(cfg, r), board_json)
        self.recipe_name = "default" if "default" in self.recipes else next(iter(self.recipes))
        self.cfg = recipe_mod.apply(cfg, self.recipes[self.recipe_name])
        self.alarms = alarm_mod.Alarms()
        self.last_judge = None
        self.robot_sh = Path(robot_sh) if robot_sh else ROOT / "scripts" / "robot.sh"
        self.data_dir = Path(data_dir) if data_dir else ROOT / cfg["station"]["data_dir"]
        self.slots = {r: Slot(r, tuple(addrs[r])) for r in roles}
        self.lock = threading.RLock()
        self.camera = threading.Lock()       # 미리보기 스냅샷 ↔ 작업
        self.connecting = threading.Lock()
        self.cancel = threading.Event()      # 링크에 넘기는 것과 같은 Event
        self.paused = threading.Event()
        self.preview_on = False
        self.job = None
        self.last_scan_dir = None
        self.last_scan_recipe = "default"
        self._log = deque(maxlen=200)
        self._stop = threading.Event()
        self._next_role = 0
        self._last_connect = 0.0
        self._worker = threading.Thread(target=self._loop, daemon=True)

    # ---- 기록 ----
    def log(self, text):
        line = f"{time.strftime('%H:%M:%S')} {text}"
        with self.lock:
            self._log.append(line)
        print(line, flush=True)

    # ---- 수명 ----
    def start(self):
        if not self.fake:                         # 로봇 구동 스택 상태를 한 번 읽어 둔다(ssh, 뒤에서)
            for r in self.slots:
                self.stack_action(r, "status")
        self.connect()
        self._worker.start()

    def close(self):
        """서버를 끌 때. 작업 중이면 abort를 보내고 작업 스레드의 정리를 기다린다."""
        self._stop.set()
        with self.lock:
            job = self.job if self.busy() else None
        if job is not None:
            job.user_cancel = True
            self.cancel.set()
            self.paused.clear()
            for s in self.slots.values():
                lk = s.link
                if lk is not None:
                    lk.abort()
            if job.thread is not None:
                job.thread.join(timeout=30.0)
        if self._worker.is_alive():               # start()가 연결 중이면 아직 안 떴다
            self._worker.join(timeout=5.0)
        for s in self.slots.values():
            self._drop(s)

    def busy(self):
        with self.lock:
            return self.job is not None and self.job.state == "running"

    # ---- 연결 ----
    def _drop(self, slot, lost=False):
        """링크를 닫는다. lost(뜻하지 않게 끊김)면 알람."""
        lk, slot.link = slot.link, None
        slot.preview = None                    # 옛 각도로 출발을 막지 않게
        slot.obstacle = None
        self.alarms.clear(f"obs:{slot.role}")
        if lost and slot.was_connected:
            self.alarms.raise_("E03", f"{slot.role}: {slot.error or '연결 끊김'}", slot.role, key=f"conn:{slot.role}")
        slot.was_connected = False
        if lk is not None:
            try:
                lk.close()
            except OSError:
                pass

    def connect(self):
        """끊긴 로봇에 연결하고 hello. 작업 중이면 Busy."""
        if self.busy():
            raise Busy("작업 중에는 다시 연결하지 않는다")
        with self.connecting:
            self._last_connect = time.monotonic()
            for s in self.slots.values():
                if s.connected() or self.busy():
                    continue
                if s.user_stopped:
                    continue                          # 화면에서 끈 로봇(켜면 다시 붙는다)
                self._drop(s)
                try:
                    lk = robot_link.connect(self.cfg, s.role, None, self.clock, self.cancel, addr=s.addr)
                except OSError as e:
                    s.error = f"연결 안 됨: {e}"
                    self.alarms.raise_("E03", f"{s.role}: {e}", s.role, key=f"conn:{s.role}")
                    continue
                try:
                    reply = lk.hello()
                    if reply.get("role") != s.role:
                        raise StartError(f"role {reply.get('role')}로 응답함(config robots 확인)")
                    lk.wait_streams()
                except Exception as e:  # noqa: BLE001  이유를 화면에 보인다
                    lk.close()
                    s.error = f"hello 실패: {e}"
                    self.alarms.raise_("E03", f"{s.role}: hello 실패 {e}", s.role, key=f"conn:{s.role}")
                    continue
                s.link, s.serial, s.arc, s.error = lk, reply.get("serial"), bool(reply.get("arc")), None
                s.was_connected = True
                self.alarms.clear(f"conn:{s.role}")
                self.log(f"{s.role} 연결: serial {s.serial}")

    # ---- 로봇 구동 스택(scripts/robot.sh) ----
    def stack_action(self, role, action):
        """robot_scan 구동 스택 켜기·끄기·상태. ssh라 몇 초 걸려 뒤에서 하고 상태로 알린다."""
        if action not in ("start", "stop", "status"):
            raise NotReady(f"모르는 동작: {action}")
        s = self.slots.get(role)
        if s is None:
            raise NotReady(f"없는 로봇: {role}")
        if self.fake:
            raise NotReady("가짜 로봇은 구동 스택이 없다")
        with self.lock:                               # 스캔 시작(_begin)과 겹치지 않게
            if action == "stop" and self.busy():
                raise Busy("작업 중에는 구동을 끄지 않는다(먼저 중지)")
            if s.stack_busy:
                raise Busy(f"{role} 구동 명령 처리 중")
            s.stack_busy = True
            if action != "status":
                s.stack, s.stack_msg = ("starting" if action == "start" else "stopping"), ""
                s.user_stopped = action == "stop"
            if action == "stop":
                self._drop(s)                         # 일부러 끊음(알람 없음). 잠금 안이라 새 작업이 이 링크를 잡지 못한다
        threading.Thread(target=self._run_stack, args=(s, action), daemon=True).start()

    def _run_stack(self, s, action):
        host = self.cfg["robots"][s.role]["host"]
        cmd = [str(self.robot_sh), "-H", host, action] + ([STACK_NAME] if action != "status" else [])
        try:
            p = subprocess.run(cmd, capture_output=True, text=True, timeout=STACK_TIMEOUT_S)
            out = (p.stdout + p.stderr).strip()
            if action == "status":
                state = "running" if f"{STACK_NAME}: running" in out else "stopped" if p.returncode == 0 else "error"
            elif action == "start":
                if p.returncode == 0 or f"{STACK_NAME} is already running" in out:
                    state = "running"
                elif "이미 구동 스택" in out:
                    state = "blocked"             # 다른 구동 스택이 LiDAR·모터를 쓰는 중
                else:
                    state = "error"
            else:
                state = "stopped" if p.returncode == 0 else "error"
            msg = out.splitlines()[-1] if out else ""
        except (OSError, subprocess.SubprocessError) as e:
            state, msg = "error", f"{type(e).__name__}: {e}"
        with self.lock:
            s.stack, s.stack_msg, s.stack_busy = state, msg[:200], False
        self.log(f"{s.role} 구동 {action}: {state}{' · ' + msg if msg else ''}")
        if action == "start" and state == "running":
            self._last_connect = time.monotonic() - RECONNECT_S + 4.0   # 에이전트가 뜰 4 s 뒤 연결
        if state in ("error", "blocked") and action != "status":
            self.alarms.raise_("E09", f"{s.role} 구동 {action}: {msg}", s.role)

    def start_stacks(self):
        """켜져 있지 않은 로봇의 구동 스택을 모두 켠다."""
        started = []
        for r, s in self.slots.items():
            if s.stack != "running" and not s.stack_busy and not s.connected():
                self.stack_action(r, "start")
                started.append(r)
        if not started:
            raise NotReady("켤 로봇이 없다(모두 켜져 있거나 연결됨)")

    # ---- 레시피 ----
    def set_recipe(self, name):
        if name not in self.recipes:
            raise NotReady(f"레시피 없음: {name}")
        with self.lock:                           # _begin과 같은 잠금(작업 설정 고정)
            if self.busy():
                raise Busy("작업 중에는 레시피를 바꾸지 않는다")
            self.recipe_name = name
            self.cfg = recipe_mod.apply(self.base_cfg, self.recipes[name])
        self.log(f"레시피: {self.recipes[name].get('label', name)}")

    def _judge(self, job, dims, scan_dir, source):
        """레시피 기준으로 OK/NG. 기준이 없으면 None. 결과는 scan_dir/judge.json에도."""
        r = self.recipes.get(job.recipe, {})
        if job.kind == "focus" and not dims:
            return self.last_judge                # 보강 메시가 비었으면 스캔 판정 유지
        j = recipe_mod.judge(dims, r)
        if j is not None:
            j.update(recipe=job.recipe, label=r.get("label", job.recipe), source=source, scan_id=job.scan_id)
            (Path(scan_dir) / "judge.json").write_text(json.dumps(j, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
            self.log(f"판정 {j['result']}: 잰 값 {j['measured']} / 기준 {j['spec']} ± {j['tol']}")
        self.last_judge = j
        return j

    # ---- 주행 이벤트(장애물) ----
    def _on_event(self, role, typ, kw):
        s = self.slots.get(role)
        if s is None or not self.busy():          # 작업이 끝난 뒤 늦게 온 이벤트는 버린다
            return
        ev = kw.get("event") if typ == "drive" else None
        if ev == "bypass":
            s.bypassing = True
        elif typ in ("leg", "view"):
            s.bypassing = False
        if ev in ("wait", "stop"):
            s.obstacle = {"event": ev, "view": kw.get("view"), "since": time.monotonic()}
            self.alarms.raise_("W06", f"{role}: 진행 방향 막힘({'정지' if ev == 'stop' else '대기'})", role,
                               key=f"obs:{role}")
        elif typ in ("leg", "view") or ev in ("go", "bypass"):
            if s.obstacle is not None:
                s.obstacle = None
                self.alarms.clear(f"obs:{role}")

    def _clear_obstacles(self):
        for r, s in self.slots.items():
            s.obstacle = None
            s.bypassing = False
            self.alarms.clear(f"obs:{r}")

    def set_obstacle(self, on, view=None):
        """가짜 시뮬레이션(--fake)에서 장애물 상자를 놓거나 치운다."""
        if not self.fake or self.fleet is None:
            raise NotReady("시뮬레이션(--fake)에서만 장애물을 바꿀 수 있다")
        if on:
            box = self.fleet.add_obstacle(k=view)
            self.log(f"장애물 추가: ({box[0]:.2f}, {box[1]:.2f})")
        else:
            self.fleet.clear_obstacles()
            self.log("장애물 제거")

    # ---- 실시간 촬영·자세 기록 ----
    def _on_capture(self, role, use, k, frame, pose, header, cfg):
        s = self.slots.get(role)
        if s is None:
            return
        mount = cfg["robots"][role]["mount"]
        values, jpeg = preview.measure_frame(frame, pose, self.board, cfg, mount)
        now = time.monotonic()
        fix = None
        shot = None
        if pose is not None:
            T = np.asarray(pose.T_cam_board, float)
            if header and header.get("odom") is not None:
                fix = (drive.base_from_cam(T, mount), tuple(header["odom"]))
            R = T[:3, :3]
            c = -R.T @ T[:3, 3]
            f = R.T @ np.array([0.0, 0.0, 1.0])
            shot = {"k": k, "use": use, "pose": True,
                    "cam": [round(float(v), 4) for v in c],
                    "look": [round(float(v), 4) for v in f],
                    "phi_deg": round(float(values["phi_deg"]), 1),
                    "d_m": round(float(values["d_m"]), 3)}
        with self.lock:
            s.preview, s.preview_t, s.jpeg, s.error = values, now, jpeg, None
            s.seq += 1
            s.last_shot_t, s.last_shot_k, s.last_shot_use = now, k, use
            if fix is not None:
                s.fix = fix
            if shot is not None and use in ("view", "retake", "focus"):
                s.shots = [x for x in s.shots if not (x["k"] == k and x["use"] == use)]
                s.shots.append(shot)
                if len(s.shots) > 96:
                    s.shots = s.shots[-96:]

    # ---- 미리보기 ----
    def set_preview(self, on):
        self.preview_on = bool(on)

    def _preview_once(self):
        roles = [r for r, s in self.slots.items() if s.connected()]
        if not roles:
            return False
        role = roles[self._next_role % len(roles)]
        self._next_role += 1
        s = self.slots[role]
        if not self.camera.acquire(blocking=False):
            return False
        try:
            lk = s.live_link()
            if lk is None:
                return False
            header, payloads, _ = lk.snapshot(None, "preview", None)
            mount = self.cfg["robots"][role]["mount"]
            frame = align.prepare(header, payloads, lk.calib)
            pose = tagpose.estimate(frame.color, frame.K, self.board)
            values, jpeg = preview.measure_frame(frame, pose, self.board, self.cfg, mount)
            fix = (drive.base_from_cam(pose.T_cam_board, mount), tuple(header["odom"])) if (
                pose is not None and header.get("odom") is not None) else None
            with self.lock:
                s.preview, s.preview_t, s.jpeg, s.error = values, time.monotonic(), jpeg, None
                s.seq += 1
                if fix is not None:
                    s.fix = fix
        except (ConnectionError, OSError, TimeoutError) as e:
            s.error = f"미리보기 실패: {e}"
            self._drop(s, lost=True)
        except Exception as e:  # noqa: BLE001
            s.error = f"미리보기 실패: {type(e).__name__}: {e}"
        finally:
            self.camera.release()
        return True

    def _sample_trails(self):
        with self.lock:
            job = self.job
        for r, s in self.slots.items():
            p = self._live_pose(r, s, job)
            if p is not None:
                self._record_trail(s, p)

    def _loop(self):
        while not self._stop.is_set():
            if self.busy():
                self._sample_trails()
                time.sleep(POLL_S)
                continue
            for s in self.slots.values():           # 작업이 없을 때 끊긴 로봇(전원, Wi-Fi)
                if s.was_connected and not s.connected():
                    s.error = s.error or "스트림 끊김"
                    self._drop(s, lost=True)
            if (any(not s.connected() for s in self.slots.values())
                    and time.monotonic() - self._last_connect > RECONNECT_S):
                try:
                    self.connect()
                except Busy:
                    pass
            if not (self.preview_on and self._preview_once()):
                time.sleep(POLL_S * 2)

    def pair_state(self):
        """두 로봇 각도 차와 출발 가능 여부. 못 잰 각도 차는 스캔이 출발 때 다시 본다."""
        tol = self.cfg["drive"]["pair_gap_tol_deg"]
        now = time.monotonic()
        phis, reasons = {}, []
        for r, s in self.slots.items():
            if not s.connected():
                reasons.append(f"{r} 연결 안 됨")
            p, t = s.preview, s.preview_t
            fresh = p and p.get("pose") and t is not None and now - t < PREVIEW_STALE_S
            phis[r] = p["phi_deg"] if fresh else None
        gap, ok, hint = preview.pair_check(phis, tol)
        if gap is not None and not ok:
            reasons.append(hint)
        return {"gap_deg": gap, "gap_ok": ok, "hint": hint, "can_start": not reasons and not self.busy(),
                "reasons": reasons}

    # ---- 작업 공통 ----
    def _hold(self, job, role=None):
        if not self.paused.is_set():
            return
        holding = job.progress.setdefault("holding", [])
        holding.append(role or "all")
        try:
            while self.paused.is_set():
                if self.cancel.is_set():
                    raise drive.Cancelled("취소됨")
                time.sleep(POLL_S / 2)
        finally:
            holding.remove(role or "all")

    def _in_recon(self, job):
        if job.kind == "scan":
            return job.pair is not None and job.pair.phase == "done"
        return job.progress.get("step") == "recon"

    def pause(self):
        with self.lock:
            job = self.job if self.busy() else None
        if job is None:
            raise NotReady("진행 중인 작업이 없음")
        if self._in_recon(job):
            raise NotReady("메시를 합치는 중이라 일시정지할 것이 없음(로봇은 서 있다)")
        self.paused.set()
        if self._in_recon(job):                    # 그 사이 재구성으로 넘어갔으면 되돌린다
            self.paused.clear()
            raise NotReady("메시를 합치는 중이라 일시정지할 것이 없음(로봇은 서 있다)")
        self.log("일시정지: 다음 단계 경계에서 선다")

    def resume(self):
        if not self.busy():
            raise NotReady("진행 중인 작업이 없음")
        self.paused.clear()
        self.log("재개")

    def cancel_job(self):
        with self.lock:
            job = self.job if self.busy() else None
        if job is None:
            raise NotReady("진행 중인 작업이 없음")
        if self._in_recon(job):
            raise NotReady("메시를 합치는 중이라 취소할 수 없음(로봇은 이미 서 있다, 곧 끝난다)")
        job.user_cancel = True
        self.cancel.set()
        self.paused.clear()
        self.log("취소: 두 로봇을 세운다")

    def _begin(self, kind):
        with self.lock:
            if self.busy():
                raise Busy("다른 작업이 진행 중")
            missing = [r for r, s in self.slots.items() if not s.connected()]
            if missing:
                raise NotReady(f"연결 안 됨: {', '.join(missing)}")
            self.cancel.clear()
            self.paused.clear()
            self.job = Job(kind, recipe=self.recipe_name, cfg=self.cfg)
            return self.job

    def _spawn(self, job, target, *args):
        job.thread = threading.Thread(target=target, args=(job, *args), daemon=True)
        job.thread.start()

    def _links(self):
        """작업이 쓸 살아 있는 링크. 하나라도 끊겼으면 NotReady."""
        links = {r: s.live_link() for r, s in self.slots.items()}
        dead = [r for r, lk in links.items() if lk is None]
        if dead:
            raise NotReady(f"연결 끊김: {', '.join(dead)}")
        return links

    def _outcome(self, job, e):
        """예외 → (state, message). 사용자가 누른 취소만 cancelled(abort_all도 cancel을 켠다). 실패면 알람."""
        if job.user_cancel and isinstance(e, (drive.Cancelled, ConnectionError)):
            return "cancelled", "취소됨"
        if not isinstance(e, (StartError, drive.Blocked, NotReady, focus.PlanError)):
            traceback.print_exc()
        message = f"{type(e).__name__}: {e}"
        self.alarms.raise_(alarm_mod.classify(e), f"{job.kind}: {message}")
        return "failed", message

    def _finish(self, job, links, state, message):
        """정리한 뒤 끝 상태를 낸다."""
        for lk in links.values():
            lk.raw = None
        self._clear_obstacles()
        if state != "done":
            for s in self.slots.values():          # 중단된 링크는 abort로 닫혔다
                self._drop(s)
            self._last_connect = 0.0               # 바로 다시 연결
        self.cancel.clear()
        self.paused.clear()
        with self.lock:
            job.ended = time.monotonic()
            job.state, job.message = state, message
        self.log(f"{job.kind} {state}{': ' + message if message else ''}")

    # ---- 스캔 ----
    def start_scan(self):
        ps = self.pair_state()
        if ps["gap_deg"] is not None and not ps["gap_ok"]:
            raise NotReady(ps["hint"])
        job = self._begin("scan")
        self.last_scan_dir = None                  # 끝나기 전에는 보강 촬영을 받지 않는다
        self._spawn(job, self._run_scan)
        return job

    def _run_scan(self, job):
        links, writer, state, message = {}, None, "failed", ""
        with self.camera:
            try:
                links = self._links()
                writer = dataset.ScanWriter(self.data_dir, self.cfg_path, self.board_path)
                job.scan_id, job.scan_dir = writer.scan_id, Path(writer.scan_dir)
                # dataset.build·pair.recon이 레시피를 덮은 설정을 읽게
                (job.scan_dir / "config.yaml").write_text(recipe_mod.dump_cfg(job.cfg), encoding="utf-8")
                (job.scan_dir / "recipe.json").write_text(json.dumps(
                    recipe_mod.public(job.recipe, self.recipes[job.recipe]), ensure_ascii=False, indent=1) + "\n",
                    encoding="utf-8")
                with self.lock:
                    for s in self.slots.values():
                        s.trail.clear()
                        s.shots = []
                        s.last_shot_t = s.last_shot_k = s.last_shot_use = None
                for r, lk in links.items():
                    lk.raw = writer.role(r)
                pair = PairScan(job.cfg, links, writer, self.board, self.cancel,
                                on_capture=lambda role, use, k, fr, po, hdr: self._on_capture(
                                    role, use, k, fr, po, hdr, job.cfg))
                pair.hold = lambda role=None: self._hold(job, role)   # 일시정지는 세션의 paused로
                record = pair.event                                      # 기록하면서 장애물 상태도 본다
                pair.event = lambda role, typ, **kw: (self._on_event(role, typ, kw), record(role, typ, **kw))
                job.pair = pair
                self.log(f"스캔 시작: {job.scan_id}")
                scan_dir, result, report = station.run(job.cfg, links, writer, self.board, self.cancel,
                                                       job.started, pair=pair)
                job.result = {"dims_mm": report.get("dims_mm"), "elapsed_s": report.get("elapsed_s"),
                              "mesh_id": f"{self.data_dir.name}/{job.scan_id}/out",
                              "skipped": report.get("skipped"), "stops": report.get("stops"),
                              "judge": self._judge(job, report.get("dims_mm"), scan_dir, "out")}
                self.last_scan_dir = Path(scan_dir)
                self.last_scan_recipe = job.recipe
                state = "done"
                d = report.get("dims_mm") or []
                message = f"{' × '.join(f'{v:.0f}' for v in d)} mm, {report.get('elapsed_s')} s"
            except BaseException as e:  # noqa: BLE001  원인을 화면에 보인다
                state, message = self._outcome(job, e)
                if job.pair is None:            # station.run 전 실패(그 뒤는 station.run이 abort_all)
                    for lk in links.values():
                        lk.abort()
            finally:
                if writer is not None:
                    writer.close()
                self._finish(job, links, state, message)

    def _scan_progress(self, job):
        pair = job.pair
        n = job.cfg["scene"]["n_views"]
        if pair is None:
            return {"phase": "hello", "holding": []}
        phase = "recon" if pair.phase == "done" and job.state == "running" else pair.phase
        robots = {r: {"view": max(u.cur, -1) + 1, "n": n, "stops": u.stats.stops,
                      "corrections": u.stats.corrections, "skipped": list(u.stats.skipped)}
                  for r, u in pair.units.items()}
        return {"phase": phase, "robots": robots, "holding": list(job.progress.get("holding", []))}

    # ---- 보강 촬영 ----
    def start_focus(self, markers=None, regions=None):
        markers, regions = markers or [], regions or []
        if not isinstance(markers, list) or not isinstance(regions, list) or not (markers or regions):
            raise focus.PlanError("markers나 regions가 한 개 이상 있어야 함")
        if self.last_scan_dir is None:
            raise NotReady("이 서버에서 끝낸 스캔이 없음(스캔 뒤 로봇과 물체를 옮기지 않은 채로 한다)")
        roles = [r for r, s in self.slots.items() if s.connected()]
        cfg = recipe_mod.apply(self.base_cfg, self.recipes[self.last_scan_recipe])   # 그 스캔에 쓴 값
        plans = focus.plan(markers, cfg, roles)
        for j, region in enumerate(regions):
            if not isinstance(region, dict):
                raise focus.PlanError(f"영역 {j}: 객체가 아님")
            plans += focus.plan_region(region, cfg, roles, len(markers) + j)
        job = self._begin("focus")
        job.scan_id, job.scan_dir = self.last_scan_dir.name, self.last_scan_dir
        job.recipe = self.last_scan_recipe
        job.progress.update(step="plan", plans=[{"role": p["role"], "point": p["point"], "phi": p["phi"],
                                                 "shots": len(p["phis"] or []) or 3} for p in plans])
        self._spawn(job, self._run_focus, plans, cfg)
        return job

    def _run_focus(self, job, plans, cfg):
        links, writers, units, state, message = {}, {}, {}, "failed", ""
        ev_lock = threading.Lock()

        def emitter(role):
            def emit(typ, **kw):
                self._on_event(role, typ, kw)
                line = json.dumps({"pc_t": self.clock.now(), "role": role, "type": typ, "focus": True, **kw},
                                  ensure_ascii=False, separators=(",", ":"))
                with ev_lock, open(job.scan_dir / "events.jsonl", "a", encoding="utf-8") as f:
                    f.write(line + "\n")
            return emit

        with self.camera:
            try:
                links = self._links()
                for r, lk in links.items():
                    writers[r] = dataset.RawWriter(job.scan_dir / r)
                    lk.raw = writers[r]
                    units[r] = RobotScan(r, lk, cfg, self.board, emitter(r),
                                         on_capture=lambda role, use, k, fr, po, hdr: self._on_capture(
                                             role, use, k, fr, po, hdr, cfg))
                    units[r].hello()
                job.units = units
                run = focus.FocusRun(cfg, units, job.scan_dir, plans,
                                     hold=lambda role=None: self._hold(job, role),
                                     cancelled=self.cancel.is_set, progress=job.progress, log=self.log)
                res = run.run()
                job.result = {"dims_mm": res["dims_mm"], "used": res["used"],
                              "judge": self._judge(job, res["dims_mm"], job.scan_dir, "out_focus"),
                              "mesh_id": f"{self.data_dir.name}/{job.scan_id}/out_focus",
                              "crops": [self._crop_json(job, i, c) for i, c in enumerate(res["crops"])]}
                state, message = "done", f"보강 사진 {len(res['crops'])}장, 다시 합친 메시 out_focus"
            except BaseException as e:  # noqa: BLE001
                for lk in links.values():               # 움직이던 로봇을 세운다
                    lk.abort()
                state, message = self._outcome(job, e)
            finally:
                for w in writers.values():
                    w.close()
                self._finish(job, links, state, message)

    @staticmethod
    def _crop_json(job, i, c):
        url = f"/api/focus/{i}.jpg?k={c['k']}&s={job.scan_id}" if c.get("file") else None
        return {**{k: v for k, v in c.items() if k != "file"}, "url": url}

    def focus_image(self, i):
        """마지막 보강 촬영 작업의 i번째 자른 사진 바이트 또는 None."""
        with self.lock:
            job = self.job
        if job is None or job.kind != "focus":
            return None
        crops = job.progress.get("crops") or []
        if not (0 <= i < len(crops)) or not crops[i].get("file"):
            return None
        path = job.scan_dir / "focus" / crops[i]["file"]
        return path.read_bytes() if path.is_file() else None

    # ---- 실시간 스테이지(3D 시각화) ----
    def _unit_of(self, job, role):
        if job is None:
            return None
        if job.pair is not None and role in job.pair.units:
            return job.pair.units[role]
        if job.units is not None and role in job.units:
            return job.units[role]
        return None

    def _live_pose(self, role, slot, job):
        """현재 로봇의 판 좌표 (x, y, yaw rad) 또는 None."""
        if self.fleet is not None:
            try:
                return tuple(float(v) for v in self.fleet.true_pose(role))
            except Exception:  # noqa: BLE001
                pass
        u = self._unit_of(job, role)
        if u is not None and u.driver._fix is not None:
            try:
                return tuple(float(v) for v in u.driver.pose())
            except Exception:  # noqa: BLE001
                pass
        lk = slot.live_link()
        fix = slot.fix
        if lk is not None and fix is not None:
            od = lk.odom()
            if od is not None:
                base, origin = fix
                offset = base[2] - origin[2]
                dx, dy = drive._rotate(od[0] - origin[0], od[1] - origin[1], offset)
                return float(base[0] + dx), float(base[1] + dy), float(drive.wrap_angle(od[2] + offset))
        return None

    def _record_trail(self, slot, pose):
        pt = (round(pose[0], 3), round(pose[1], 3))
        with self.lock:
            if not slot.trail or math.hypot(pt[0] - slot.trail[-1][0], pt[1] - slot.trail[-1][1]) >= 0.015:
                slot.trail.append(pt)

    def _lidar_points(self, pose, lk, cfg):
        """최신 LiDAR 스캔을 판 좌표 [[x, y], …]로 변환(최대 약 90점)."""
        if pose is None or lk is None:
            return []
        sc, age = lk.scan()
        if sc is None or age > cfg["drive"]["scan_timeout_s"] * 2.0:
            return []
        ranges = sc["ranges"]
        n = len(ranges)
        if not n:
            return []
        step = max(1, n // 90)
        start, inc = sc["angle_min"], sc["angle_inc"]
        rmin, rmax = max(0.05, sc["range_min"]), min(2.2, sc["range_max"])
        mount_yaw = math.radians(cfg["lidar"]["mount_yaw_deg"])
        mask = cfg["lidar"].get("mask") or []
        rx, ry, ryaw = pose
        pts = []
        for i in range(0, n, step):
            r = ranges[i]
            if not (rmin <= r <= rmax) or not math.isfinite(r):
                continue
            ang_robot = start + i * inc + mount_yaw
            if mask and drive.in_mask(math.degrees(ang_robot), mask):
                continue
            ang = ryaw + ang_robot
            pts.append([round(rx + r * math.cos(ang), 3), round(ry + r * math.sin(ang), 3)])
        return pts

    def _stage_state(self, now, job):
        cfg = (job.cfg if job is not None and job.state == "running" and job.cfg else self.cfg)
        sc = cfg["scene"]
        d = cfg["drive"]
        holding = set((job.progress.get("holding") or []) if job is not None and job.state == "running" else [])
        robots = {}
        for r, s in self.slots.items():
            lk = s.live_link()
            pose = self._live_pose(r, s, job)
            if pose is not None and job is not None and job.state == "running":
                self._record_trail(s, pose)
            mount = cfg["robots"][r]["mount"]
            cam = None
            if pose is not None:
                cx, cy = drive.cam_xy(pose, mount)
                cam = [round(cx, 4), round(cy, 4), round(float(mount["z"]), 4)]
            u = self._unit_of(job, r)
            if s.obstacle is not None:
                activity = "waiting"
            elif s.bypassing:
                activity = "bypassing"
            elif r in holding or "all" in holding:
                activity = "holding"
            elif u is not None and job is not None and job.state == "running":
                activity = u.activity
            else:
                activity = "idle"
            with self.lock:
                trail = [list(p) for p in s.trail]
                shots = list(s.shots)
                shot_t, shot_k, shot_use = s.last_shot_t, s.last_shot_k, s.last_shot_use
            robots[r] = {
                "pose": None if pose is None else [round(pose[0], 4), round(pose[1], 4), round(pose[2], 4)],
                "cam": cam,
                "mount": {"x": float(mount["x"]), "y": float(mount["y"]), "z": float(mount["z"]),
                          "yaw_deg": float(mount["yaw_deg"]), "pitch_down_deg": float(mount["pitch_down_deg"])},
                "activity": activity,
                "shot_age_s": None if shot_t is None else round(now - shot_t, 2),
                "shot_k": shot_k,
                "shot_use": shot_use,
                "trail": trail,
                "shots": shots,
                "lidar": self._lidar_points(pose, lk, cfg),
            }
        obstacles = []
        target_box = None
        if self.fleet is not None:
            try:
                from ..sim.fake_agent import BOX
                target_box = list(BOX)
                obstacles = [[round(float(v), 4) for v in b] for b in self.fleet.world_boxes()]
            except Exception:  # noqa: BLE001
                pass
        return {
            "board_m": round(float(self.board_json.get("board_mm", 594.0)) / 1000.0, 4),
            "tags": [{"id": t["id"], "corners": [[round(c[0], 4), round(c[1], 4)] for c in t["corners"]]}
                     for t in self.board_json.get("tags", [])],
            "scene": {
                "r_obj": float(sc["r_obj"]),
                "r_eff": float(sc["r_obj"]) + float(sc["place_err"]),
                "obj_h": float(sc["obj_h"]),
                "R": float(sc["R"]),
                "n_views": int(sc["n_views"]),
                "bypass_lane": float(d["bypass_lane"]),
                "sector_half_deg": float(d["sector_half_deg"]),
                "start_clear": float(d["start_clear"]),
                "stop_dist": float(d["stop_dist"]),
            },
            "target_box": target_box,
            "obstacles": obstacles,
            "robots": robots,
        }

    # ---- 화면 상태 ----
    def state(self):
        now = time.monotonic()
        robots = {}
        for r, s in self.slots.items():
            lk = s.live_link()
            ages = lk.ages() if lk is not None else (None, None)
            front = None
            if lk is not None:
                sc, age = lk.scan()
                if sc is not None and age <= self.cfg["drive"]["scan_timeout_s"]:
                    front = round(min(drive.front_min(sc, age, self.cfg), 9.99), 3)
            obs = s.obstacle
            pv, pt = s.preview, s.preview_t
            pv = dict(pv) if pv else None
            if pv is not None and pt is not None:
                pv["age_s"] = round(now - pt, 1)
            mount = self.cfg["robots"][r]["mount"]
            robots[r] = {"addr": f"{s.addr[0]}:{s.addr[1]}", "connected": lk is not None, "serial": s.serial,
                         "arc": s.arc, "odom_age_s": _r(ages[0]), "scan_age_s": _r(ages[1]),
                         "error": s.error, "preview": pv, "preview_seq": s.seq,
                         "pitch_cfg_deg": mount["pitch_down_deg"], "z_cfg_m": mount["z"], "front_m": front,
                         "obstacle": None if obs is None else {"event": obs["event"], "view": obs["view"],
                                                               "since_s": round(now - obs["since"], 1)},
                         "stack": "sim" if self.fake else ("running" if lk is not None and s.stack == "unknown"
                                                           else s.stack),
                         "stack_msg": s.stack_msg, "stack_busy": s.stack_busy}
        with self.lock:
            job = self.job
            log = list(self._log)[-40:]
        d = self.cfg["drive"]
        fs = focus.settings(self.cfg)
        out = {"mode": "control", "fake": self.fake, "robots": robots, "pair": self.pair_state(), "preview_on": self.preview_on,
               "job": None, "last_scan": None, "log": log,
               "limits": {"d_min": d["d_min"], "d_max": d["d_max"], "gap_tol_deg": d["pair_gap_tol_deg"],
                          "n_views": self.cfg["scene"]["n_views"], "split_z": fs["split_z"],
                          "up_deg": fs["up_deg"], "start_clear": d["start_clear"], "stop_dist": d["stop_dist"]},
               "recipe": {"current": recipe_mod.public(self.recipe_name, self.recipes[self.recipe_name]),
                          "list": [recipe_mod.public(n, r) for n, r in self.recipes.items()]},
               "alarms": self.alarms.snapshot(), "judge": self.last_judge,
               "stage": self._stage_state(now, job)}
        if self.last_scan_dir is not None:
            out["last_scan"] = {"scan_id": self.last_scan_dir.name,
                                "mesh_id": f"{self.data_dir.name}/{self.last_scan_dir.name}/out"}
        if job is not None:
            running = job.state == "running"
            if job.kind == "scan":
                progress = self._scan_progress(job)
            else:
                pr = job.progress
                progress = {"phase": pr.get("step") or "plan", "target": pr.get("target", 0),
                            "n_targets": pr.get("n_targets", 0), "shots": pr.get("shots", 0),
                            "holding": list(pr.get("holding", [])), "plans": pr.get("plans"),
                            "crops": [self._crop_json(job, i, c) for i, c in enumerate(pr.get("crops") or [])]}
            out["job"] = {"kind": job.kind, "state": job.state, "paused": running and self.paused.is_set(),
                          "message": job.message, "scan_id": job.scan_id, "progress": progress,
                          "recon": running and self._in_recon(job),
                          "elapsed_s": round((job.ended or now) - job.started, 1), "result": job.result}
        return out


def _r(v):
    return None if v is None else round(v, 2)

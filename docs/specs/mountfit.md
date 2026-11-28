# mountfit — 카메라 장착 위치·방향을 주행하면서 표식으로 재기

`scan3d/robot/mountfit.py`(numpy만), `scan3d/robot/unit.py`의 `_fit_mount`, config `drive.mount_fit`.

로봇 기준점(바퀴 축 가운데)의 자세는 표식으로 잰 카메라 자세에서 장착값 `mount`(x, y, yaw_deg)를 빼서 구한다.
장착값이 실제와 다르면 로봇이 가는 방향과 도는 중심을 틀리게 알아, 원호가 원 안팎으로 벗어나고 거리 보정이 잦아진다.
config의 `mount` x·y·yaw_deg는 처음 값으로만 쓰고, 원호를 달릴 때마다 표식으로 다시 맞춘다. z·pitch_down_deg는 그대로 둔다.

## 재는 것

원호 문장 `arc r θ`를 달리면 차체는 바퀴 축의 연장선 위, 기준점에서 왼쪽으로 r(θ < 0이면 오른쪽으로 r) 떨어진 점을 중심으로 돈다.
원호 앞뒤의 표식 자세 두 개(카메라 위치와 보는 수평 방향)에서 그 회전 중심을 구해 원호 앞 카메라 기준(보는 방향 x, 왼쪽 y)으로
적은 것이 관측 `q`다. 장착값과의 관계:

    q = Rot(−yaw) · (−x, r_s − y)        r_s: 부호 있는 반지름(왼쪽으로 돌면 +r)

- `planar(T_cam_board) -> (x, y, heading)`: 카메라의 판 좌표 위치와 보는 수평 방향
- `arc_centre(T_before, T_after) -> (q, 돈 각) | None`: 돈 각이 5° 미만이거나 150° 초과면 None
- `observe(T_before, T_after, odom_yaw_before, odom_yaw_after, radius, angle_deg) -> (q, r_s) | None`:
  명령과 반대로 돌았거나, 표식으로 잰 회전각이 odom 회전각과 2.5° 넘게 다르면(미끄러짐, 표식 자세가 튐) None
- `predict(mount, r_s) -> q`

## 맞추기

`MountFit(prior)`: 최근 관측 24개에 `q = b + r_s·ŷ`(b: 카메라 기준 기준점 위치, ŷ: 로봇 왼쪽 방향)를 최소제곱으로 맞춘다.

- ŷ = (Σ(r − r̄)(q − q̄) + λ·ŷ₀) / (Σ(r − r̄)² + λ)를 단위 벡터로, λ = 0.05 m², ŷ₀는 처음 값의 yaw.
  반지름이 고루 다르면(제곱합이 λ보다 큼) yaw가 관측으로 정해지고, 반지름이 비슷하면 처음 값의 yaw에 머문다.
- b = q̄ − r̄·ŷ. yaw = atan2(ŷ.x, ŷ.y), (x, y) = −Rot(yaw)·b.
- 관측이 4개 이상이면 맞춘 뒤 4 cm(또는 중앙값의 3배) 넘게 벗어난 관측을 빼고 한 번 더 맞춘다.
- 맞춘 값이 |x|·|y| 0.25 m, yaw가 처음 값에서 25° 밖이면 그 관측을 버리고 지난 값을 둔다. `add()`가 False.
- 반지름이 같은 원호 하나만 있어도 그 반지름에서 도는 중심은 맞는다(x·y가 yaw 오차를 메운다). 그래서 x와 yaw는 실제 값과 다르게 나올 수 있다.

## 쓰는 곳(`RobotScan`)

- `hello()`: `drive.mount_fit`이 참이면 이 프로세스가 그 카메라 일련번호로 앞서 맞춘 값(`recall`)에서, 없으면 config 값에서 시작한다.
  웹 화면 서버는 스캔에서 맞춘 값을 보강 촬영과 다음 스캔이 이어 쓴다. 서버를 새로 띄우면 config 값에서 다시 시작한다.
- 표식으로 자세를 잡을 때마다(`_fix`): 지난 표식 자세 뒤로 보낸 이동 문장(`Driver.take_sent`)이 끝까지 달린 원호 하나뿐이면
  `observe` → `MountFit.add` → 바뀌었으면 `RobotScan.mount`와 `Driver.mount`를 새 dict로 바꾸고, 그 값으로 기준점 자세를 구한다.
  회전 + 직진, 거리 보정, 우회, 도중에 선 원호, 표식이 안 보인 스냅샷이 낀 구간은 쓰지 않는다. `cfg`의 dict는 고치지 않는다.
- 바뀔 때마다 이벤트 `mount`(x, y, yaw_deg, n)를 `events.jsonl`에 남긴다.
- config의 처음 값이 실제와 멀면 처음 두세 원호는 그만큼 벗어난다(실물 H: 설계값에서 시작하면 첫 원호의 회전 중심 예측이 14 cm,
  둘째 8 cm, 그 뒤 3 cm 안팎 틀림).

## 완료 테스트(`tests/test_mountfit.py`)

| 테스트 | 통과 조건 |
|---|---|
| `test_arc_centre` | 합성 원호(왼쪽·오른쪽, 장착값 세 가지)에서 q가 `predict`와 같음, 2° 원호는 None |
| `test_fit_recovers_mount` | 반지름 0.41 ~ 0.87 m 원호 8개, 표식 위치 잡음 1 mm, 틀린 처음 값 → x·y 1 cm, yaw 1.5° 안 |
| `test_fit_one_radius_keeps_yaw` | 반지름이 같은 원호 하나 → yaw는 처음 값, 그 반지름의 q는 맞음 |
| `test_observe_gates` | odom 회전각과 3.5° 다르면 None, 명령과 반대로 돌면 None, 오른쪽 원호는 음수 반지름 |
| `test_fit_rejects` | NaN·한도 밖 관측은 버리고 값이 그대로, 튄 관측 하나는 빼고 맞춤 |
| `test_lap_from_wrong_mount` | 2D 주행, 실제 장착이 config와 다른 H·L(실물에서 잰 값), 처음 값은 설계값 → 카메라가 원에서 6 cm 안, 시점 7부터 1.2 cm 안, 시점 5부터 회전 문장 없음 |
| `test_scan_with_wrong_mount`(slow) | 가짜 로봇 H의 카메라를 다른 자리에 두고 한 바퀴 → `mount` 이벤트 15개 이상, 도는 중심 2 cm 안, 시점 8부터 `d`가 목표 ± 1.5 cm, 거리 보정 1회 이하 |

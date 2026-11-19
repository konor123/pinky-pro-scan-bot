# Pinky Solar — 개발 계획서 v3.0

**Pinky Pro 기반 Negative Obstacle Detection 및 Vision-based Autonomous Navigation**

- 개발 환경: Ubuntu 24.04 / ROS 2 Jazzy
- 플랫폼: Pinky Pro
- 실험 대상: 실내 모형 태양광 패널
- 핵심 기술: Monocular Visual SLAM, Sensor Fusion, Negative Obstacle Detection, Coverage Path Planning

---

## 1. 프로젝트 개요

### 1.1 개발 목표

Pinky Pro의 기본 RGB 카메라 한 대를 사용하여 LiDAR 없이 태양광 모형 패널 위에서 자율주행하는 로봇을 구현한다.

핵심 기술 과제는 다음과 같다.

**A. Negative Obstacle Detection**

패널 끝단, 틈새, 단차와 같이 주행 가능한 지지면이 사라지는 영역을 영상으로 검출하고, 로봇의 추락을 방지한다.

**B. Vision-based Localization**

Monocular Visual SLAM과 Wheel Odometry·IMU를 융합하여 로봇의 위치와 방향을 추정한다.

**C. Autonomous Coverage Navigation**

패널의 유효 작업 영역을 계산하고, 경계에 접근하면 감속·정지·회전하여 왕복 주행을 반복한다.

청소 장치는 구현하지 않으며, 가상 청소 폭을 기준으로 Coverage를 평가한다.

### 1.2 확정 개발 조건

| 항목 | 확정 사항 |
|---|---|
| 로봇 | Pinky Pro |
| 카메라 | 기본 5MP RGB 카메라 1대 |
| 추가 Depth Camera | 사용하지 않음 |
| LiDAR | 사용하지 않음 |
| IMU | BNO055 |
| Odometry | Wheel Encoder 기반 |
| Visual SLAM | ORB-SLAM3 Monocular |
| Sensor Fusion | Wheel + IMU EKF, Visual SLAM 전역 보정 |
| Marker | AprilTag, 위치 정확도 검증용 |
| 실험 환경 | 실내 모형 태양광 패널 |
| 경로 계획 | Boustrophedon Coverage |
| Navigation | ROS 2 Nav2 또는 자체 Waypoint Follower |

### 1.3 최종 성공 조건

- 기본 카메라만으로 패널 경계 검출
- 정지 가능 거리 내 접근 전 안전 정지
- Visual SLAM의 Metric Scale 확보
- Wheel/IMU와 SLAM을 결합한 실시간 위치 추정
- AprilTag 기반 위치 정확도 평가
- 패널 유효 작업 영역의 90% 이상 Coverage
- 카메라·SLAM·통신 이상 시 안전 정지

---

## 2. 선행연구 분석

### 2.1 핵심 참고논문

**Esuga-Mopah, D. A. et al. (2026)**

*Negative obstacle detection and avoidance using YOLOv8 and depth profile analysis for autonomous mobile navigation.*

Discover Applied Sciences, 8, Article 305.

https://doi.org/10.1007/s42452-026-08326-5

이 연구는 YOLOv8n, RGB-D 카메라 및 2D LiDAR를 결합하여 Negative Obstacle을 검출하고 ROS 2 Nav2에서 회피할 수 있도록 표현하는 방법을 제시하였다.

#### 논문의 알고리즘

1. RGB 영상에서 사다리꼴 ROI 설정
2. YOLOv8n으로 Negative Obstacle 후보 검출
3. Depth Profile Analysis로 후보 검증
4. 검출된 경계의 3D 좌표 계산
5. Negative Obstacle을 Virtual Obstacle로 변환
6. Nav2 Costmap에 반영
7. 경로 재계획 및 장애물 회피

#### 논문의 주요 성능

| 항목 | 보고 결과 |
|---|---|
| YOLOv8n mAP50 | 0.954 |
| YOLOv8n mAP50–95 | 0.669 |
| 결합 알고리즘 처리율 | 약 27 FPS |
| YOLO 단독 False Detection Rate | 44.78% |
| YOLO + Depth False Detection Rate | 33.96% |
| 일관된 감지 거리 | 0.3~0.8m |
| 검증 환경 | Gazebo 시뮬레이션 |

연구용 데이터셋은 2,062장으로 구성되며 Train/Validation/Test를 70/20/10으로 분할하였다.

해당 성능은 논문의 시뮬레이션 및 RGB-D·LiDAR 융합 조건에서 측정된 값이다.

### 2.2 공개 소스코드

**Repository**

https://github.com/OxyBloom/negativeobstacleavoidandance

**핵심 구현 파일**

https://github.com/OxyBloom/negativeobstacleavoidandance/blob/main/bot_camera/bot_camera/clean_yolo.py

저장소에는 다음 구성 요소가 포함되어 있다.

- `bot_camera`: YOLO 및 영상 처리
- `bumperbot_bringup`: 로봇 실행 환경
- `bumperbot_controller`: 구동 제어
- `bumperbot_localization`: 위치 추정 및 Navigation
- `bumperbot_mapping`: 지도 작성
- `realsense_gazebo_plugin`: RGB-D 카메라 시뮬레이션

`clean_yolo.py`에는 ROI 처리, YOLO 추론, Depth Profile Analysis, 좌표 변환 및 가상 장애물 발행이 구현되어 있다.

공개 코드의 핵심 노드는 Depth 영상을 필수 입력으로 사용하고, 가상 장애물을 `sensor_msgs/PointCloud2`로 발행한다. 논문에서 설명한 Virtual LiDAR 방식과 구별하여 실제 코드 구조를 참고한다.

또한 공개 노드의 처리 Timer는 0.2초 간격이므로, 논문의 약 27 FPS와 동일한 실행 처리율을 보장하는 구현으로 간주하지 않는다.

### 2.3 Pinky Solar 적용 전략

| 논문의 구성 요소 | Pinky Solar 적용 |
|---|---|
| Trapezoidal ROI | 적용 |
| YOLOv8n Detection | 비교 알고리즘으로 적용 |
| Depth Profile Analysis | 단안 RGB로 대체 불가 |
| RGB-D 기반 거리 계산 | 평면 Homography로 대체 |
| Virtual Obstacle 생성 | 적용 |
| PointCloud2 발행 | 구조 참고 |
| LiDAR Scan Fusion | 제외 |
| Nav2 Costmap 연동 | 적용 |
| 경로 재계획 | 적용 |
| Detection Ablation Study | 적용 |

**핵심 변경점**

논문은 실제 Depth 값으로 Negative Obstacle을 검증한다.

Pinky Solar는 Depth Camera가 없으므로 다음 두 가지 정보를 결합한다.

1. **Appearance Evidence:** 패널과 비주행 영역 사이의 영상 특징
2. **Geometric Evidence:** 알려진 패널 평면과 카메라의 상대 자세, 연속 프레임의 기하학적 일관성

검출된 패널 경계를 지도에 비주행 영역으로 등록한다.

단안 영상만으로 실제 깊이의 단차를 직접 측정하는 것이 아니라, 주행 지지면의 시각적 경계를 검출하는 방식이다.

---

## 3. 하드웨어 구성 및 실험 환경

### 3.1 Pinky Pro 구성

| 장치 | 역할 |
|---|---|
| Raspberry Pi SBC | Onboard 제어 |
| 기본 RGB Camera | 경계 감지 및 SLAM |
| BNO055 IMU | 각속도·자세 |
| Wheel Odometry | 이동량·속도 |
| Dynamixel 구동계 | 차동구동 |
| RPLiDAR C1 | 비활성화 |
| IR·Ultrasonic | 기본 알고리즘에서 제외 |

### 3.2 연산 분담

**Onboard — Pinky Pro**

- RGB 영상 취득
- OpenCV 기반 경계 감지
- 경계 거리 계산
- Safety Supervisor
- Wheel/IMU 입력
- 구동 명령 제한
- Command Timeout 정지

**Offboard — Ubuntu PC**

- ORB-SLAM3
- Visual Trajectory 처리
- Scale Alignment
- Localization Manager
- YOLOv8n 비교 실험
- Coverage Planner
- Nav2
- RViz2 및 rosbag2

안전 제어에 필요한 경계 감지는 Onboard에서 실행한다.

YOLOv8n은 우선 PC에서 실행하여 기존 OpenCV 방식과 비교하고, 처리 속도와 감지 성능을 확인한 후 Onboard 적용 가능성을 평가한다.

### 3.3 기본 카메라 검증

카메라를 추가하지 않는다.

D1~D3에 다음 값을 실측한다.

- 실제 Image Resolution 및 FPS
- Horizontal/Vertical FOV
- 카메라 장착 높이
- Camera Pitch
- 카메라 영상에서 패널 경계가 처음 관측되는 거리
- 유효 전방 검출 거리
- ORB Feature 수 및 추적 유지율

**카메라 가시성과 안전 정지 조건을 동시에 만족해야 한다.**

필요하면 기존 카메라의 장착 각도를 조절하되, 추가 카메라를 사용하지 않는다.

### 3.4 모형 패널 설계

초기 실험장 규격:

| 항목 | 설계값 |
|---|---|
| 전체 영역 | 2,400 × 1,200mm |
| 표면 | 태양광 패널 유사 반사성 표면 |
| 패턴 | 반복적인 셀 무늬 |
| 단차 | 교체 가능한 낮은 단차 구조 |
| 틈새 | 폭 조절 가능한 모형 |
| Marker | 패널 주변 고정 구조물 |
| 초기 주행 속도 | 0.05m/s |
| 최대 시험 속도 | 0.10m/s |

실험장은 다음 단계로 구성한다.

- Level 1: 바닥에 표시한 가상 패널
- Level 2: 단차와 틈새가 있는 모형
- Level 3: 실제 끝단 접근 및 자율주행

---

## 4. Negative Obstacle Detection

### 4.1 알고리즘 개발 방향

두 가지 검출 알고리즘을 독립적으로 구현하고 비교한다.

**Method A — Classical Computer Vision**

- Color/Intensity Normalization
- Canny Edge Detection
- Line Segment Detection
- Contour Detection
- RANSAC
- Temporal Tracking

**Method B — YOLOv8n**

- 논문의 ROI 설계 참고
- 모형 패널 데이터셋 구축
- 패널 끝단·틈새 Bounding Box 학습
- Detection Confidence 평가
- Geometric Validation

두 방법은 동일한 시험 영상과 Ground Truth를 사용하여 비교한다.

### 4.2 데이터셋 구축

실제 Pinky Pro의 기본 카메라로 수집한 영상을 사용한다.

초기 목표:

| 항목 | 수량 |
|---|---:|
| 학습 영상 | 700장 |
| 검증 영상 | 150장 |
| 시험 영상 | 150장 |
| 합계 | 1,000장 |

수집 조건:

- 정면·사선 경계
- 다양한 카메라-경계 거리
- 틈새 크기 변화
- 강한 반사광
- 그림자
- 조도 변화
- 셀 패턴과 실제 경계가 혼동되는 장면
- Negative Obstacle이 없는 장면

CVAT를 이용해 Bounding Box를 작성한다.

데이터셋은 개별 프레임을 무작위로 나누지 않고 촬영 세션·조명 조건·장면 단위로 분리한다. 동일한 연속 영상의 인접 프레임이 학습과 시험에 동시에 포함되지 않도록 한다.

### 4.3 ROI Processing

논문의 Trapezoidal ROI 개념을 적용한다.

로봇 진행 방향에 대응하는 영상 영역만 우선 분석하여 처리량을 줄인다.

단, 카메라 시야와 주행 방향에 따라 ROI를 수정하며 다음 영역을 평가한다.

- 중앙 전방
- 좌전방·우전방
- 차체 폭에 해당하는 위험 영역
- 회전 시 진입할 수 있는 영역

카메라 뒤쪽과 같이 관측되지 않은 영역은 안전하다고 가정하지 않는다.

### 4.4 Boundary Geometry

패널을 평면으로 가정한다.

Camera Calibration으로 Intrinsic Matrix \(K\), Camera-to-Base Extrinsic을 구한다.

패널 평면 좌표 \((X,Y)\)와 영상 좌표 \((u,v)\) 사이의 Homography:

\[
s\begin{bmatrix}u\\v\\1\end{bmatrix}
=
H\begin{bmatrix}X\\Y\\1\end{bmatrix}
\]

\[
H=K[r_1\quad r_2\quad t]
\]

검출된 경계가 패널 평면 위에 있다는 기하학적 조건을 사용하여 영상에서의 경계 위치를 실제 거리로 변환한다.

Homography는 패널 외부 영역 전체의 3D 깊이를 복원하는 데 사용하지 않는다.

### 4.5 Temporal Validation

프레임 간 경계 검출 결과의 일관성을 평가한다.

- 이전 경계선 위치
- Wheel/IMU 이동량
- 영상상의 경계 이동
- 검출 신뢰도
- 거리 추정 오차
- 연속 검출 여부

OpenCV 검출 결과와 YOLO 결과를 비교하여 경계 후보의 안정성을 평가한다.

검출이 일시적으로 실패하면 기존 추정값의 불확실성을 증가시키며, 안전성이 확인되지 않은 상태에서는 주행을 제한한다.

### 4.6 Virtual Obstacle 생성

논문의 Negative-to-Positive Obstacle 변환 개념을 활용한다.

변환 과정:

1. 영상에서 경계선 검출
2. Homography로 패널 평면 좌표 계산
3. 경계점 집합 생성
4. `base_link` 또는 `map` 좌표계로 변환
5. 위험 영역 Polygon 구성
6. Nav2 Costmap에 반영

주요 표현:

- `geometry_msgs/PolygonStamped`: 경계 표현
- `nav_msgs/OccupancyGrid`: 패널 지도와 비주행 영역
- `sensor_msgs/PointCloud2`: 필요한 경우 동적 장애물 인터페이스
- Costmap Keepout 영역: 경계 및 외부 구역

**실제 LiDAR 데이터를 사용하지 않고도 가상의 장애물 정보를 Navigation에 전달한다.**

정적인 패널 외곽은 Keepout 영역으로 유지하고, 새로 관측한 위험 경계는 동적으로 갱신한다.

### 4.7 안전 정지

안전 정지 거리는 다음과 같이 정의한다.

\[
d_{\mathrm{stop}}=
v\tau+\frac{v^2}{2a_{\min}}+3\sigma_d+m
\]

| 기호 | 의미 |
|---|---|
| \(v\) | 현재 진행 속도 |
| \(\tau\) | 감지·제어 지연 |
| \(a_{\min}\) | 최소 감속도 |
| \(\sigma_d\) | 경계 거리 추정 불확실성 |
| \(m\) | 안전 여유 거리 |

초기 안전 여유 거리 \(m\)은 50mm로 설정하고 실험 결과에 따라 조정한다.

정지 거리 계산에는 카메라 처리 시간뿐 아니라 통신 및 모터 감속 시간이 포함된다.

경계 거리의 기준점은 로봇 중심이 아니라 진행 방향의 Footprint 최외곽점이다.

### 4.8 Detection Ablation Study

논문의 Ablation Study 방식을 참고한다.

| 실험군 | 검출 방법 |
|---|---|
| A | OpenCV 단독 |
| B | YOLOv8n 단독 |
| C | OpenCV + Temporal Validation |
| D | YOLOv8n + Geometric Validation |
| E | OpenCV + YOLOv8n 결합 |

평가 항목:

- Precision
- Recall
- False Positive Rate
- False Negative Rate
- Boundary Distance Error
- Processing Latency
- Effective FPS
- 안전 정지 성공률

최종 기본 알고리즘은 감지율뿐 아니라 지연, 오검출 및 Onboard 실행 가능성을 함께 고려하여 선정한다.

---

## 5. Vision-based Localization

### 5.1 위치 추정 아키텍처

두 계층으로 구분한다.

**Local Odometry**

Wheel Encoder + IMU → EKF

**Global Localization**

ORB-SLAM3 Monocular → Scale Alignment → Map Pose Correction

로봇의 연속적인 이동 추정은 Local EKF가 담당하고, Visual SLAM은 전역 위치 오차 보정을 담당한다.

### 5.2 Wheel Odometry

차동구동 운동 모델을 사용한다.

\[
\Delta s=\frac{\Delta s_R+\Delta s_L}{2}
\]

\[
\Delta\theta=\frac{\Delta s_R-\Delta s_L}{b}
\]

좌우 바퀴의 이동량과 Wheel Separation을 이용하여 로봇의 이동량을 계산한다.

직선 및 회전 시험을 통해 Wheel Radius와 유효 Wheel Separation을 보정한다.

### 5.3 Wheel + IMU EKF

`robot_localization`의 EKF를 적용한다.

초기 설정:

| 항목 | 설정 |
|---|---|
| `two_d_mode` | true |
| `frequency` | 30Hz |
| `world_frame` | odom |
| `odom_frame` | odom |
| `base_link_frame` | base_link |

입력:

- Wheel Linear Velocity
- Wheel Angular Velocity
- IMU Angular Velocity
- 검증된 경우 IMU Orientation

출력:

- `/odometry/filtered`
- `odom → base_link`

### 5.4 ORB-SLAM3 Monocular

ORB-SLAM3를 기본 Visual SLAM 엔진으로 선정한다.

주요 처리 과정:

1. ORB Feature Extraction
2. Feature Matching
3. Visual Tracking
4. Keyframe 생성
5. Local Mapping
6. Loop Detection
7. Loop Closure
8. Camera Pose 출력

패널 표면은 반복 패턴이 많으므로 다음 성능을 확인한다.

- 특징점 개수
- 특징점 공간 분포
- Inlier Ratio
- Tracking Lost Rate
- Loop Closure 안정성

주변 고정 구조물의 비반복 특징점을 함께 활용한다.

### 5.5 Monocular Scale Alignment

단안 SLAM의 Scale Ambiguity를 Wheel Odometry의 Metric Reference로 보정한다.

\[
p_m=sR\hat p_v+t
\]

여기서 \(s\)는 Metric Scale이다.

구현 순서:

1. SLAM과 Wheel Odometry의 시간 동기화
2. 충분한 병진 이동 구간 수집
3. 두 Trajectory 사이의 Similarity Alignment
4. Scale 추정
5. 이동 구간에 따른 Scale 안정성 검증
6. SLAM Pose를 Metric Coordinate로 변환

Pure Rotation 구간에서는 Scale을 새로 추정하지 않는다.

Wheel Slip이 발생한 구간은 Scale Estimation에서 제외한다.

### 5.6 TF 구성

```text
map
 └── odom
      └── base_link
           ├── camera_link
           │    └── camera_optical_frame
           └── imu_link
```

TF 역할:

| Transform | 담당 |
|---|---|
| `map → odom` | Visual SLAM Localization Manager |
| `odom → base_link` | Local EKF |
| `base_link → camera_link` | Robot Description |
| `base_link → imu_link` | Robot Description |

SLAM에서 추정한 Camera Pose를 Base Pose로 변환한다.

\[
T^m_b=T^m_c(T^b_c)^{-1}
\]

이후 Local Odometry와 결합하여 Map-to-Odom 변환을 계산한다.

\[
T^m_o=T^m_b(T^o_b)^{-1}
\]

Loop Closure 보정은 `map → odom`에 반영한다.

### 5.7 Localization Fallback

| 상태 | 제어 |
|---|---|
| SLAM 정상 | 전역 위치 보정 |
| Feature 부족 | 불확실성 증가 |
| SLAM 일시 중단 | Wheel/IMU 사용 |
| Scale 불안정 | 전역 경로 주행 제한 |
| 장시간 Tracking Lost | 정지 및 재위치 추정 |
| Local Odometry 이상 | 안전 정지 |

위치 추정 실패와 경계 검출 실패는 별도 상태로 관리한다.

---

## 6. AprilTag 기반 위치 정확도 검증

### 6.1 검증 방식

AprilTag를 패널 주변 고정 구조물에 배치한다.

Marker의 실제 크기와 설치 위치를 측정하고, 카메라 관측으로 로봇의 기준 Pose를 계산한다.

Marker 관측값은 기본 SLAM 위치 추정에 입력하지 않는다.

Marker 영역이 SLAM Feature Matching 결과에 영향을 줄 수 있으므로 검증 모드에서는 Marker Masking을 적용한다.

### 6.2 Ground Truth

Marker의 Map Pose를 \(T^m_t\), 카메라에서 관측한 Marker Pose를 \(T^c_t\)로 정의하면,

\[
T^m_b=
T^m_t(T^c_t)^{-1}(T^b_c)^{-1}
\]

로 로봇의 기준 Pose를 구할 수 있다.

### 6.3 비교 대상

| 그룹 | 구성 |
|---|---|
| A | Wheel Odometry |
| B | Wheel + IMU EKF |
| C | Visual SLAM + Wheel/IMU |

### 6.4 평가 지표

- Absolute Trajectory Error (ATE)
- Relative Pose Error (RPE)
- Position RMSE
- Heading RMSE
- Loop Closure Error
- Tracking Lost Rate
- Relocalization Time
- Scale Error

\[
RMSE_{xy}=
\sqrt{
\frac{1}{N}
\sum_{i=1}^{N}
[(x_i-\hat x_i)^2+(y_i-\hat y_i)^2]
}
\]

Marker가 관측되는 구간에서 위치 정확도를 측정한다.

Marker 기반 평가 결과와 SLAM Trajectory를 같은 Map Coordinate에 표시하고 오차를 시각화한다.

---

## 7. Coverage Path Planning

### 7.1 패널 지도

패널의 외곽을 Polygon으로 정의한다.

\[
P=\operatorname{Polygon}(p_1,p_2,p_3,p_4)
\]

경계 안전 여유와 로봇 Footprint를 고려한 내부 주행 영역을 생성한다.

### 7.2 Coverage Path

Boustrophedon Coverage를 적용한다.

1. 패널의 유효 주행 영역 계산
2. 가상 청소 폭 설정
3. 중첩률을 반영한 평행 경로 생성
4. 직선 주행
5. 끝단에서 감속·정지
6. 안전 영역 내 방향 전환
7. 다음 경로로 이동
8. 전체 영역 반복

초기 설계값:

- 가상 청소 폭: 100mm
- 경로 간격: 80mm
- 목표 Coverage: 90% 이상

### 7.3 Navigation

초기에는 자체 Waypoint Follower로 기본 경로를 검증한다.

이후 Nav2의 Costmap과 Path Follower를 통합한다.

Negative Obstacle Detection의 결과는 Costmap의 비주행 영역으로 반영한다.

경로 계획이 유효하더라도 Safety Supervisor가 위험을 감지하면 해당 구동 명령을 차단한다.

카메라 시야 밖의 방향으로 후진하거나 회전할 때는 기존에 확인한 안전 영역과 로봇의 Swept Footprint를 기준으로 제어한다.

---

## 8. ROS 2 소프트웨어 구성

### 8.1 패키지 구조

```text
pinky_solar_ws/
└── src/
    ├── pinky_pro/
    │
    ├── pinky_solar_bringup/
    │   ├── launch/
    │   └── config/
    │
    ├── pinky_solar_vision/
    │   ├── camera_preprocessor
    │   ├── panel_segmenter
    │   ├── yolo_edge_detector
    │   ├── boundary_tracker
    │   └── boundary_projector
    │
    ├── pinky_solar_slam/
    │   ├── orbslam3_adapter
    │   ├── scale_aligner
    │   └── slam_monitor
    │
    ├── pinky_solar_localization/
    │   ├── localization_manager
    │   └── marker_evaluator
    │
    ├── pinky_solar_safety/
    │   ├── boundary_monitor
    │   ├── safety_supervisor
    │   └── cmd_vel_gate
    │
    ├── pinky_solar_navigation/
    │   ├── panel_map
    │   ├── coverage_planner
    │   └── mission_manager
    │
    └── pinky_solar_interfaces/
        └── msg/
```

### 8.2 참고 코드 활용

논문의 공개 구현인 `bot_camera/bot_camera/clean_yolo.py`를 참고하여 다음 기능을 개발한다.

| 공개 코드 기능 | 프로젝트 모듈 |
|---|---|
| `ROIProcessor` | `camera_preprocessor` |
| `YoloDetectorNode` | `yolo_edge_detector` |
| `CameraCalibration` | `boundary_projector` |
| `DepthAnalyzer` | RGB 기반 `boundary_tracker`로 대체 |
| PointCloud2 Publisher | `virtual_obstacle_publisher` |
| Navigation Integration | `panel_map` / Nav2 |

RGB 입력과 YOLO Detection 부분은 재구성하되 Depth 입력을 필수로 사용하는 함수는 제거한다.

Geometry Validation과 Temporal Tracking을 새로 구현한다.

### 8.3 주요 Topic

| Topic | Message |
|---|---|
| `/camera/image_raw` | Image |
| `/camera/camera_info` | CameraInfo |
| `/imu/data` | Imu |
| `/wheel/odom` | Odometry |
| `/odometry/filtered` | Odometry |
| `/slam/pose` | PoseStamped |
| `/panel/boundary` | PolygonStamped |
| `/panel/edge_distance` | Custom Message |
| `/virtual_obstacles` | PointCloud2 |
| `/localization/pose` | PoseWithCovarianceStamped |
| `/safety/state` | Custom Message |
| `/cmd_vel_nav` | Twist |
| `/cmd_vel_safe` | Twist |

위 Topic명은 프로젝트용 인터페이스로 정의하고 실제 Pinky Pro 드라이버 Topic에 Remapping한다.

모터 제어에는 Safety Supervisor가 허용한 `cmd_vel_safe`만 전달한다.

---

## 9. Safety Supervisor

### 9.1 상태 정의

| State | 조건 |
|---|---|
| IDLE | 초기 대기 |
| LOCALIZING | 위치 추정 초기화 |
| SWEEPING | 정상 경로 주행 |
| EDGE_APPROACH | 경계 접근 |
| TURNING | 안전 방향 전환 |
| RELOCALIZING | 위치 추정 복구 |
| STOPPED | 안전 정지 |
| FAULT | 센서·통신·제어 이상 |

### 9.2 제어 우선순위

**Safety Supervisor → Navigation Controller → Mission Manager**

안전 제어가 Navigation보다 우선한다.

다음 조건에서는 구동 명령을 차단한다.

- 경계까지의 여유 거리 부족
- 카메라 프레임 중단
- 경계 검출 불확실성 과다
- 구동 명령 갱신 중단
- 필수 Odometry 입력 중단
- 허용 범위를 초과한 위치 추정 불확실성

시스템은 정상 주행 명령이 없는 상태에서도 정지 상태를 유지하도록 구성한다.

---

## 10. 개발 일정

**15작업일 + 예비 5작업일**

| 단계 | 일정 | 개발 항목 | 완료 기준 |
|---|---|---|---|
| P1 | D1 | Pinky Bringup 및 센서 확인 | 영상·IMU·Odom 수신 |
| P2 | D2 | Camera Calibration | Intrinsic/Extrinsic 확보 |
| P3 | D3 | 패널 제작 및 FOV 시험 | 경계 관측 가능 거리 확보 |
| P4 | D4 | OpenCV Boundary Detection | 직선 경계 검출 |
| P5 | D5 | YOLOv8n 비교 구현 | Detection 결과 출력 |
| P6 | D6 | Geometry·Temporal Validation | 경계 거리 안정화 |
| P7 | D7 | Safety Supervisor | 자동 감속·정지 |
| P8 | D8 | Wheel/IMU EKF | Local Odometry |
| P9 | D9~D10 | ORB-SLAM3 연동 | Visual Tracking |
| P10 | D11 | Scale Alignment | Metric Pose 추정 |
| P11 | D12 | AprilTag 평가 | 위치 오차 비교 |
| P12 | D13 | Coverage Planner | 왕복 경로 생성 |
| P13 | D14 | Navigation 통합 | 자동 왕복 주행 |
| P14 | D15 | 최종 평가 | 전체 시연 성공 |
| 예비 | D16~D20 | 성능 개선 | 미달 항목 보완 |

### 주요 Milestone

**M1 — Camera & Environment Ready**

- 실측 FOV 확보
- Camera Calibration
- 패널 경계 관측 조건 확인

**M2 — Negative Obstacle Detection**

- OpenCV / YOLO 비교
- 거리 추정
- 안전 감속·정지

**M3 — Vision-based Localization**

- Wheel/IMU EKF
- ORB-SLAM3
- Metric Scale Alignment
- AprilTag 정확도 평가

**M4 — Autonomous Coverage**

- Coverage Path
- 경계 감속 및 방향 전환
- 반복 왕복 주행
- 정량 성능 평가

---

## 11. 시험 계획 및 성공 기준

### 11.1 Negative Obstacle Detection

| Test | 시험 조건 | 평가 |
|---|---|---|
| N01 | 정면 경계 | Recall |
| N02 | 사선 경계 | Recall |
| N03 | 틈새 폭 변화 | 검출 가능 폭 |
| N04 | 그림자 | False Positive |
| N05 | 반사광 | False Positive |
| N06 | 조도 변화 | Detection Robustness |
| N07 | 카메라 중단 | 정지 반응 |
| N08 | 속도 변화 | 정지 거리 |
| N09 | 회전 중 경계 접근 | Footprint 안전성 |

성공 기준:

- 경계 감지 Recall ≥ 95%
- 경계 판정 지연 P95 ≤ 200ms
- 검증 조건 내 안전 영역 이탈 0회
- 영상 중단 시 자동 정지
- 검출 거리와 실제 거리의 오차 측정 및 기록

각 주요 조건을 최소 30회 반복한다.

논문의 검출 성능을 참고하되, RGB 단안 환경에서 독립적으로 성능을 평가한다.

### 11.2 Localization

| Test | 평가 내용 |
|---|---|
| L01 | 1m 직선 이동 오차 |
| L02 | 90° 회전 오차 |
| L03 | 사각 경로 종료 오차 |
| L04 | 반복 패턴 Tracking |
| L05 | 조도 변화 Tracking |
| L06 | Loop Closure |
| L07 | Tracking Lost Fallback |
| L08 | Relocalization |

성공 기준:

- Position RMSE ≤ 50mm
- Heading RMSE ≤ 5°
- Tracking 유지율 ≥ 95%
- Wheel/IMU 대비 Visual SLAM 융합 ATE 개선
- SLAM 실패 시 안전한 제어 전환

실제 Accuracy는 AprilTag로 관측한 기준 Pose를 사용해 계산한다.

### 11.3 Coverage Navigation

| Test | 평가 내용 |
|---|---|
| C01 | 직선 경로 추종 |
| C02 | 경계 감속 |
| C03 | 자동 회전 |
| C04 | 다음 경로 진입 |
| C05 | 전체 Coverage |
| C06 | 위치 추정 실패 복구 |

성공 기준:

- 유효 작업 영역 Coverage ≥ 90%
- 전체 Mission 성공률 ≥ 90%
- 위험 경계 이탈 0회
- 자동 경로 전환 성공

전체 Mission은 최소 10회 반복하여 평가한다.

### 11.4 실험 데이터 기록

rosbag2로 다음 데이터를 수집한다.

- Camera Image
- CameraInfo
- IMU
- Wheel Odometry
- EKF Odometry
- Visual SLAM Pose
- AprilTag Ground Truth
- Boundary Detection
- Safety State
- Command Velocity
- Timestamp 및 Processing Latency

실험군별로 동일한 영상 데이터와 평가 기준을 적용한다.

---

## 12. 최종 데모

### 시나리오

1. 모형 태양광 패널 위에 Pinky Pro 배치
2. Camera / Wheel / IMU 초기화
3. Visual SLAM 초기 위치 추정
4. 패널 영역 인식
5. Coverage Path 생성
6. 첫 번째 주행 경로 진입
7. 경계 검출 및 자동 감속
8. 안전 거리 확보 후 정지
9. 다음 경로로 방향 전환
10. Coverage 반복
11. 작업 종료 및 정지

### 시각화

RViz2 또는 프로젝트 대시보드에 다음 정보를 표시한다.

- RGB Camera View
- 검출된 패널 경계
- Virtual Obstacle
- SLAM Map
- Wheel/IMU Trajectory
- Visual SLAM Trajectory
- AprilTag Ground Truth
- Planned Coverage Path
- 실제 이동 경로
- Coverage 진행률
- Safety State

---

## 13. 최종 산출물

1. Pinky Solar ROS 2 Workspace
2. Camera Calibration 결과
3. 모형 태양광 패널 실험 환경
4. OpenCV Boundary Detector
5. YOLOv8n Boundary Detector
6. Geometry / Temporal Validation
7. Virtual Obstacle Generator
8. Safety Supervisor
9. Wheel/IMU EKF
10. ORB-SLAM3 ROS 2 Adapter
11. Monocular Scale Alignment
12. AprilTag Position Evaluator
13. Coverage Path Planner
14. Nav2 통합 패키지
15. Negative Obstacle Detection 비교 평가
16. Localization ATE/RPE 평가
17. Coverage 결과
18. 최종 주행 시연 영상 및 발표자료

---

## 14. 참고문헌 및 기술 자료

### 14.1 핵심 연구논문

**[1] Esuga-Mopah, D. A., Shonde, O. H., Maiti, M., DasMahapatra, S., & Santra, S. (2026).**

*Negative obstacle detection and avoidance using YOLOv8 and depth profile analysis for autonomous mobile navigation.*

Discover Applied Sciences, 8, Article 305.

DOI: https://doi.org/10.1007/s42452-026-08326-5

**활용:** ROI 설계, YOLOv8n Negative Obstacle Detection, Virtual Obstacle 표현, Nav2 통합, Ablation Study.

### 14.2 논문 공개 소스코드

**[2] Esuga-Mopah et al. — Negative Obstacle Detection and Avoidance Research Code.**

GitHub Repository:

https://github.com/OxyBloom/negativeobstacleavoidandance

핵심 구현 파일:

https://github.com/OxyBloom/negativeobstacleavoidandance/blob/main/bot_camera/bot_camera/clean_yolo.py

**활용:** YOLO Detector, Trapezoidal ROI, 좌표 변환, PointCloud2 기반 Virtual Obstacle 생성, ROS 2 Navigation 연동.

### 14.3 로봇 플랫폼

**[3] PinkLAB — Pinky Pro**

https://pinklab.art/pinky-pro/

**[4] PinkLAB — Pinky Pro ROS 2 Packages**

https://github.com/pinklab-art/pinky_pro

### 14.4 Visual SLAM 및 Localization

**[5] ORB-SLAM3**

https://github.com/UZ-SLAMLab/ORB_SLAM3

**[6] ROS 2 ORB-SLAM3 Integration Reference**

https://github.com/Mechazo11/ros2_orb_slam3

**[7] AprilTag ROS 2**

https://github.com/christianrauch/apriltag_ros

**[8] robot_localization**

https://docs.ros.org/en/ros2_packages/jazzy/api/robot_localization/

### 14.5 Navigation 및 Safety

**[9] Nav2 — Navigation 2**

https://docs.nav2.org/

**[10] Nav2 — Keepout Filter**

https://docs.nav2.org/tutorials/docs/navigation2_with_keepout_filter.html

**[11] Nav2 — Collision Monitor**

https://docs.nav2.org/configuration/packages/collision_monitor/configuring-collision-monitor-node.html

### 14.6 영상 처리 및 학습

**[12] OpenCV**

https://docs.opencv.org/4.x/

**[13] Ultralytics YOLO**

https://docs.ultralytics.com/

**[14] CVAT**

https://docs.cvat.ai/

---

## 15. 핵심 개발 전략

본 프로젝트의 핵심은 단일 RGB 카메라를 두 가지 목적에 동시에 사용하는 것이다.

- Negative Obstacle Detection은 패널의 **주행 가능 경계**를 추정한다.
- Visual SLAM은 주변의 **고정 특징점**을 이용하여 로봇의 위치를 추정한다.
- Wheel Odometry·IMU는 영상 기반 추정의 불연속과 누적 오차를 보완한다.
- Virtual Obstacle Generator는 경계 인식 결과를 Navigation에 전달한다.
- Safety Supervisor는 Navigation과 독립적으로 주행 안전을 관리한다.

**최종 목표는 LiDAR 없이 Negative Obstacle을 감지하고, 비전 기반 위치 추정으로 태양광 패널의 유효 작업 영역을 자율주행하는 것이다.**
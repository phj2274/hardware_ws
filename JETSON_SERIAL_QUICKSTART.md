# Jetson Serial Quickstart (1-page)

이 문서는 "다른 사람이 Jetson에서 ESP32와 시리얼 통신을 바로 시작"하기 위한 최소 절차입니다.

## 1. 준비물

- Jetson (Ubuntu + ROS 2 Humble 환경)
- ESP32 보드
- USB-UART 또는 Jetson UART 핀
- 3.3V 레벨 TX/RX/GND 배선

주의:
- TX/RX는 교차 연결 (Jetson TX -> ESP32 RX, Jetson RX -> ESP32 TX)
- GND는 반드시 공통 연결
- 5V TTL 직접 연결 금지

## 2. 코드 받기

```bash
cd ~
git clone <YOUR_REPO_URL> hardware_ws
cd hardware_ws
```

## 3. ROS 의존성 설치

ROS 2 Humble이 이미 설치되어 있다는 전제입니다.

```bash
source /opt/ros/humble/setup.bash
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

`ackermann_msgs`가 누락되면:

```bash
sudo apt install ros-humble-ackermann-msgs
```

## 4. 빌드

```bash
cd ~/hardware_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select motor_serial_bridge_cpp
source install/setup.bash
```

## 5. ESP32 펌웨어 업로드

아래 파일을 ESP32에 업로드:
- `esp32/esp32_motor_servo_bridge/esp32_motor_servo_bridge.ino`

Jetson 브리지와 ESP32 펌웨어의 baud rate를 동일하게 맞추세요 (기본 115200).

펌웨어 기본 핀맵:
- UART2 RX/TX: GPIO16 / GPIO17
- 모터 PWM/DIR/ENA: GPIO27 / GPIO26 / GPIO14
- 서보: GPIO33
- 엔코더 A/B: GPIO18 / GPIO19

## 6. 시리얼 포트 설정

기본 설정 파일:
- `src/motor_serial_bridge_cpp/config/motor_serial_bridge.yaml`

대표 포트 예시:
- Jetson UART: `/dev/ttyTHS1` 또는 `/dev/ttyAMA0`
- USB UART: `/dev/ttyUSB0`

포트 확인:

```bash
ls /dev/ttyTHS* /dev/ttyAMA* /dev/ttyUSB* 2>/dev/null
```

권한 문제(`Permission denied`)가 나면:

```bash
sudo usermod -a -G dialout $USER
newgrp dialout
```

## 7. 실행

```bash
cd ~/hardware_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch motor_serial_bridge_cpp motor_serial_bridge.launch.py
```

커스텀 설정 파일 사용:

```bash
ros2 launch motor_serial_bridge_cpp motor_serial_bridge.launch.py \
  config:=/absolute/path/to/motor_serial_bridge.yaml
```

## 8. 빠른 점검

토픽 확인:

```bash
ros2 topic list
```

브리지 출력 확인:

```bash
ros2 topic echo /diagnostics
ros2 topic echo /odom
```

명령 테스트(예시):

```bash
ros2 topic pub /ackermann_cmd ackermann_msgs/msg/AckermannDriveStamped \
"{drive: {speed: 0.5, steering_angle: 0.1}}" -1
```

UART 프로토콜 참고:
- Jetson -> ESP32:
  - `CMD spd=0.50 steer=0.10 estop=0`
  - `CMD pwm=1600 steer_us=1700 estop=0`
- ESP32 -> Jetson:
  - `FB vel_mps=... steer=... fault=0 batt=0.0 rpm=... duty=... age_ms=... estop=...`

확장 필드 의미:
- `rpm`: 엔코더 기반 모터 RPM(EMA 필터 적용)
- `duty`: 현재 모터 PWM 듀티(부호 포함)
- `age_ms`: 마지막 명령 이후 경과 시간
- `estop`: ESP32 내부 estop latch 상태

속도 폐루프 파라미터 기본값(ESP32):
- `R=0.034 m`  // TODO: 실측 필요
- `G=1.0`      // TODO: 감속비 확인
- `CPR=11`, `QUAD_FACTOR=4` // TODO: 엔코더 스펙/배선에 맞게 확인
- `Kp=0.3`, `Ki=0.8` // TODO: 실차 튜닝
- `MAX_MOTOR_RPM=4000` // TODO: 7.4V 실측값으로 갱신

튜닝 절차:
1. `Ki=0`으로 시작하고 `Kp`만 올려 step 응답 확인
2. `Ki`를 점진적으로 추가해 steady-state error 제거
3. 출력 포화 구간에서 anti-windup 동작 확인
4. slew rate(`PWM_RAMP_STEP`, `MAX_DUTY_DELTA_PER_CYCLE`)를 조정해 슬립 억제

검증 체크리스트:
- [ ] 공회전에서 `rpm_meas`가 `rpm_ref`를 추종
- [ ] 지면 접촉 상태에서 0.3m/s, 2.0m/s 안정 동작
- [ ] 300ms timeout 시 모터 정지 + 서보 중립 복귀
- [ ] estop 수신 시 즉시 정지 + 적분기 리셋
- [ ] 후진 명령에서 DIR 핀/엔코더 부호 일치
- [ ] 서보 중립 직진, 좌/우 끝점 동작 정상
- [ ] `FB` 확장 필드(rpm/duty/age_ms/estop) 수신 정상

## 9. 자주 막히는 포인트

- 포트 이름 불일치: YAML의 `port`와 실제 장치명 확인
- baud mismatch: Jetson/ESP32 양쪽 동일 값 설정
- 배선 오류: TX/RX 교차, GND 공통 여부 확인
- 권한 문제: `dialout` 그룹 추가 후 셸 재시작
- 펌웨어 미업로드: ESP32가 `CMD ...` / `FB ...` 프로토콜을 처리해야 함

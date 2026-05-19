#!/usr/bin/env python3
"""
cmd_vel 릴레이 노드 — /joystick_cmd → /j100_0001/cmd_vel 중계기

[이 파일이 하는 일]
  Node 1(joystick_publisher)이 발행하는 "이미 스케일링된" Twist 를
  /joystick_cmd 토픽에서 구독하고, 그것을 로봇이 실제로 듣는
  /j100_0001/cmd_vel 토픽으로 그대로 다시 발행(republish)합니다.

  단순한 통과(pass-through)가 아니라 다음 두 가지를 더 합니다.
    1. 입력 주기와 무관하게 항상 20Hz(50 ms)로 출력 → 출력 레이트 보장.
    2. watchdog: 입력이 0.5 s 이상 끊기면 zero Twist 를 능동적으로 발행
       → joystick_publisher 가 죽거나 멈춰도 로봇이 확실히 정지.
  바로 이 두 가지가 "릴레이 노드를 굳이 따로 두는 진짜 이유" 입니다.

[2-노드 데이터 흐름도]

      ┌────────────────────┐   geometry_msgs/msg/Twist
      │ Node 1             │        (이미 스케일링됨)
      │ joystick_publisher │ ───────────────────────────┐
      └────────────────────┘                            │
                                                        ▼
                                              토픽: /joystick_cmd
                                                        │
                                                        ▼
      ┌────────────────────┐                            │
      │ Node 2             │ ◀──────────────────────────┘  (구독)
      │ cmd_vel_relay      │
      │  (이 파일)          │ ───────────────────────────┐  (재발행)
      └────────────────────┘                            │
                                                        ▼
                                          토픽: /j100_0001/cmd_vel
                                                        │
                                                        ▼
                                       Clearpath J100 로봇 / clearpath_gz 시뮬레이터

[ROS2 노드 정보]
  노드 이름 : cmd_vel_relay
              → 실행 후 `ros2 node list` 를 치면 /cmd_vel_relay 가 보입니다.
  구독 토픽 : /joystick_cmd          (geometry_msgs/msg/Twist)
  발행 토픽 : /j100_0001/cmd_vel     (geometry_msgs/msg/Twist)

[실행 방법]
  $ source /opt/ros/humble/setup.bash        # ROS2 환경 설정 (한 번만)
  $ source install/setup.bash                # 패키지 빌드 후 오버레이 설정
  $ ros2 run j100_teleop cmd_vel_relay        # 이 릴레이 노드 단독 실행
  (Node 1 인 joystick_publisher 도 별도 터미널에서 함께 실행되어야 의미가 있습니다)

────────────────────────────────────────────────────────────────────────────
[이 노드의 핵심 — 멀티스레딩(MultiThreadedExecutor) 설계]
────────────────────────────────────────────────────────────────────────────

이 릴레이 노드는 이 연습의 핵심인 "멀티스레딩"을 반드시 사용합니다.
SingleThreadedExecutor 로 폴백하지 않습니다(요구사항상 금지).

● SingleThreadedExecutor vs MultiThreadedExecutor
  - SingleThreadedExecutor : 모든 콜백(구독 콜백, 타이머 콜백 등)을
      단 하나의 스레드에서 "순차적으로" 실행합니다. 즉, 구독 콜백이
      실행되는 동안에는 타이머 콜백이 절대 실행되지 못하고 줄을 서서 기다립니다.
  - MultiThreadedExecutor : 여러 스레드를 두고 콜백을 "병렬로" 실행할
      수 있습니다. 단, 병렬 실행이 실제로 일어나려면 콜백들이 서로 다른
      "콜백 그룹(callback group)" 에 속해 있어야 합니다.

● MutuallyExclusiveCallbackGroup 이 하는 일
  하나의 MutuallyExclusiveCallbackGroup 에 들어 있는 콜백들은
  (MultiThreadedExecutor 라 하더라도) 동시에 둘 이상 실행되지 않습니다.
  즉 "그룹 내부에서는 직렬, 그룹끼리는 병렬" 입니다.

● 왜 그룹을 2개로 따로 두는가
  - cb_sub  : 구독(/joystick_cmd) 콜백 전용 그룹.
  - cb_timer: 20Hz 타이머(_tick_relay) 콜백 전용 그룹.
  이 둘을 "서로 다른" MutuallyExclusiveCallbackGroup 에 넣었기 때문에
  구독 콜백과 타이머 콜백이 서로 다른 스레드에서 "동시에" 돌 수 있습니다.
  만약 둘을 같은 그룹(혹은 기본 그룹)에 두면 MultiThreadedExecutor 를
  써도 둘이 직렬화되어, 입력이 한 번 늦거나 튀면(jitter) 그동안
  cmd_vel 출력 타이머까지 같이 막혀 출력 레이트가 무너집니다.
  → 우리는 "느리거나 불규칙한 입력이 일정한 cmd_vel 출력을 막지 않고,
     반대로 출력 타이머가 입력 수신을 막지도 않도록" 두 콜백을
     독립적으로(병렬로) 돌리고 싶습니다. 이것이 바로
     "무겁고 독립적인 콜백들을 분리해 서로를 막지 않게 한다"의 구체적 구현입니다.

● 왜 threading.Lock 이 필수인가
  바로 위에서 구독 콜백과 타이머 콜백을 일부러 "다른 스레드에서 동시에"
  돌게 만들었기 때문에, 두 콜백이 같은 공유 상태(최신 Twist, 마지막 수신
  시각)에 동시에 접근하면 데이터 레이스(경쟁 상태)가 발생합니다.
  한쪽이 쓰는 도중 다른 쪽이 읽으면 찢어진(torn) 값을 볼 수 있습니다.
  그래서 공유 상태 접근을 threading.Lock 으로 감싸 직렬화합니다.
  Lock 이 필요한 이유는 다름 아닌 "우리가 MultiThreadedExecutor +
  분리된 콜백 그룹을 선택했기 때문" 이라는 점이 핵심입니다.
"""
# ── 표준 라이브러리 ────────────────────────────────────────────────────────
import threading  # 두 콜백이 공유 상태를 동시 접근하므로 Lock 이 필요
import time        # watchdog 경과시간 측정에 time.monotonic() 사용 (아래 설명)

# ── ROS2 클라이언트 라이브러리(rclpy) ─────────────────────────────────────
import rclpy                                    # ROS2 파이썬 바인딩의 핵심 모듈
from geometry_msgs.msg import Twist             # 구독/발행 모두 쓰는 메시지 타입
# MutuallyExclusiveCallbackGroup: 같은 그룹 내 콜백은 동시 실행되지 않음(그룹 간엔 병렬).
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
# MultiThreadedExecutor: 콜백을 여러 스레드에서 병렬 실행 (이 연습의 필수 요구사항).
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node                     # 모든 ROS2 노드의 기반 클래스
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
# QoSProfile : 퍼블리셔/서브스크라이버의 통신 품질(Quality of Service) 설정 모음


# ── 전역 상수 ─────────────────────────────────────────────────────────────

# Node 1(joystick_publisher)이 발행하고, 이 노드가 구독하는 토픽 이름.
#   앞에 '/'가 붙으면 절대 경로이므로 어떤 네임스페이스에서 실행해도
#   항상 이 이름 그대로 구독합니다. Node 1 의 발행 토픽과 반드시 일치해야 합니다.
JOY_CMD_TOPIC = '/joystick_cmd'

# 이 노드가 재발행하여 로봇/시뮬레이터가 실제로 구독하는 토픽 이름.
#   '/j100_0001/' 부분은 Clearpath J100 로봇의 네임스페이스(robot_id)로,
#   clearpath_gz 시뮬레이터/실 로봇의 robot.yaml 에서 설정됩니다.
#   확인: `ros2 topic list | grep cmd_vel`
CMD_VEL_TOPIC = '/j100_0001/cmd_vel'

# 재발행 주기: 50 ms = 20 Hz.
#   cmd_vel 을 일정 시간 못 받으면 로봇이 안전을 위해 자동 정지하는
#   watchdog(twist_mux timeout 기본값 0.5 s) 이 있으므로,
#   그보다 훨씬 자주(20 Hz) 보내야 로봇이 계속 움직입니다.
#   일반적으로 cmd_vel 은 10 ~ 50 Hz 범위가 적합합니다.
#   너무 느리면 watchdog 에 걸려 정지, 너무 빠르면 CPU/네트워크 낭비.
RELAY_PERIOD_S = 0.05           # 20 Hz, well under twist_mux 0.5 s timeout

# 입력 watchdog 타임아웃: 마지막으로 /joystick_cmd 를 받은 지 이 시간이
#   넘으면(또는 아직 한 번도 못 받았으면) zero Twist 를 능동적으로 발행합니다.
#   "아무것도 안 보내는" 것이 아니라 0 을 능동적으로 보내야 watchdog 없이도
#   로봇이 확실히 정지 상태를 유지합니다. 0.5 s 는 twist_mux 기본 timeout 과
#   동일하게 잡되, 우리는 그보다 먼저 능동 정지를 보냅니다.
INPUT_TIMEOUT = 0.5             # seconds; joystick_publisher 가 멈추면 능동 정지

# ★★ watchdog 시계는 반드시 time.monotonic() — ROS clock 을 쓰면 안 됨 ★★
#   watchdog 의 핵심 계산은 "지금 - 마지막 수신시각 > INPUT_TIMEOUT" 입니다.
#   이 '경과 시간' 측정에는 절대 뒤로 가지 않는 단조(monotonic) 시계가 필수입니다.
#
#   self.get_clock().now() 를 쓰면 안 되는 이유:
#     · 기본값(RCL_ROS_TIME)은 시스템 '벽시계(wall clock)' 를 따라갑니다.
#       NTP 보정이나 수동 시간 변경으로 시계가 '뒤로' 점프하면
#       (now - last_rx) 가 음수가 되어 watchdog 이 영원히 안 걸립니다
#       → 입력이 끊겼는데도 로봇이 마지막 명령으로 계속 달리는 안전사고.
#     · 이 패키지는 Clearpath/Gazebo 시뮬레이션 환경에서도 쓰입니다.
#       만약 use_sim_time 이 켜지면 ROS clock 은 '시뮬레이션 시간' 이 되는데,
#       시뮬을 일시정지하면 sim clock 이 멈춰 watchdog 도 같이 얼어붙습니다
#       → 입력이 끊겨도 zero 를 못 보냄.
#   time.monotonic() 은 OS 가 보장하는 단조 증가 시계로,
#   벽시계 점프·sim-time 정지와 무관하게 항상 앞으로만 흐릅니다.
#   안전 watchdog 의 경과시간 측정에는 이것이 정답입니다.
#   (절대 시각이 필요한 게 아니라 '두 시점의 차이' 만 필요하므로 monotonic 으로 충분.)

# 상태 로그 출력 주기(초). 매 tick(50 ms)마다 로그를 찍으면 너무 시끄러우므로
#   약 2초에 한 번만 현재 상태(정상 중계 / watchdog 정지 / 구독자 없음)를 출력합니다.
LOG_PERIOD_S = 2.0


# ────────────────────────────────────────────────────────────────────────
# ROS2 node
# ────────────────────────────────────────────────────────────────────────
class CmdVelRelay(Node):
    """/joystick_cmd 를 구독해 /j100_0001/cmd_vel 로 재발행하는 릴레이 노드.

    rclpy.node.Node 를 상속받아 ROS2 노드로서의 기능(퍼블리셔, 서브스크라이버,
    타이머, 로깅 등)을 모두 사용합니다.

    [멀티스레딩 요점 — 클래스 차원]
      구독 콜백(_on_joy_cmd)과 타이머 콜백(_tick_relay)을 서로 다른
      MutuallyExclusiveCallbackGroup 에 배정하여, MultiThreadedExecutor
      아래에서 두 콜백이 다른 스레드에서 동시에 실행되도록 합니다.
      그래서 두 콜백이 공유하는 상태(_latest_twist, _last_rx_time)는
      반드시 self._lock(threading.Lock) 으로 보호합니다.
    """

    def __init__(self):
        # ── ROS2 노드 초기화 ──────────────────────────────────────────────
        # super().__init__('cmd_vel_relay') 는 부모 클래스인 Node 의 __init__ 을 호출합니다.
        # 인자로 전달한 문자열 'cmd_vel_relay' 가 이 노드의 ROS2 그래프상 이름이 됩니다.
        # 실행 후 터미널에서 `ros2 node list` 를 입력하면 /cmd_vel_relay 로 나타납니다.
        # 노드 이름은 ROS2 그래프에서 고유해야 하며, 같은 이름의 노드가 두 개 실행되면
        # 충돌이 발생할 수 있습니다.
        super().__init__('cmd_vel_relay')

        # ── QoS(통신 품질) 설정 ──────────────────────────────────────────
        # QoSProfile 은 퍼블리셔와 서브스크라이버가 메시지를 어떻게 주고받을지 결정합니다.
        # cmd_vel 처럼 "최신 값만 중요하고 과거 메시지는 필요 없는" 토픽에는 아래 설정이 적합합니다.
        #
        # ★★ 가장 흔한 함정(#1 gotcha): QoS 불일치 ★★
        #   Node 1(joystick_publisher)의 퍼블리셔가 BEST_EFFORT 로 발행하는데
        #   여기 구독 QoS 를 RELIABLE 로 두면, ROS2(DDS)는 두 엔드포인트가
        #   "호환되지 않는다"고 판단하여 연결 자체를 맺지 않습니다.
        #   그러면 에러는 안 뜨는데 데이터가 단 한 개도 안 들어옵니다(silent no-data).
        #   → 즉 구독 QoS 의 reliability 는 반드시 BEST_EFFORT 여야
        #     Node 1 의 BEST_EFFORT 퍼블리셔와 호환됩니다.
        #   (RELIABLE 구독 + BEST_EFFORT 발행 = 호환 불가. 디버깅이 매우 어려우니 주의!)
        #   확인: `ros2 topic echo /joystick_cmd` 가 조용하면 QoS부터 의심.
        qos = QoSProfile(
            # BEST_EFFORT: 전송 실패 시 재전송하지 않음. 실시간성을 우선시합니다.
            # (반대: RELIABLE — 재전송 보장, 지연 가능성 있음)
            # 여기서는 Node 1 의 BEST_EFFORT 퍼블리셔와 호환되도록 BEST_EFFORT 필수.
            reliability=ReliabilityPolicy.BEST_EFFORT,
            # VOLATILE: 새로 연결한 서브스크라이버에게 과거 메시지를 보내지 않음.
            durability=DurabilityPolicy.VOLATILE,
            # KEEP_LAST + depth=10: 최대 10개의 메시지만 큐에 보관합니다.
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            # depth(큐 크기)=10: 서브스크라이버가 처리 속도가 느릴 때 최대 10개까지
            # 메시지를 버퍼에 쌓아 둡니다. cmd_vel 처럼 빠른 토픽은 10이면 충분합니다.
        )

        # ── 콜백 그룹 2개 생성 (멀티스레딩의 핵심) ────────────────────────
        # MutuallyExclusiveCallbackGroup: 같은 그룹 안의 콜백들은 (멀티스레드
        # executor 라도) 동시에 둘 이상 실행되지 않습니다. 그룹이 "다르면"
        # 비로소 서로 다른 스레드에서 병렬 실행될 수 있습니다.
        #
        #   cb_sub   → 구독(/joystick_cmd) 콜백 전용.
        #   cb_timer → 20Hz 타이머(_tick_relay) 콜백 전용.
        #
        # 이 둘을 "서로 다른" 그룹에 배정했기 때문에 입력 수신(구독)과
        # 일정 출력(타이머)이 독립된 스레드에서 동시에 굴러갑니다.
        # → 입력이 잠깐 늦거나 튀어도 cmd_vel 출력 20Hz 가 흔들리지 않고,
        #   반대로 출력 타이머가 바빠도 입력 수신을 막지 않습니다.
        # (만약 같은 그룹/기본 그룹에 두면 MultiThreadedExecutor 를 써도
        #  둘이 직렬화되어 이 분리의 효과가 사라집니다.)
        self.cb_sub = MutuallyExclusiveCallbackGroup()    # 구독 콜백에 배정
        self.cb_timer = MutuallyExclusiveCallbackGroup()  # 타이머 콜백에 배정

        # ── 공유 상태 + Lock ─────────────────────────────────────────────
        # _latest_twist  : 가장 최근에 /joystick_cmd 로 받은 Twist (없으면 None).
        # _last_rx_time   : 그 Twist 를 받은 시각(초, time.monotonic 기준 —
        #                   watchdog 안전성을 위해 ROS clock 이 아니라 단조 시계).
        #
        # 이 두 변수는 _on_joy_cmd(구독 스레드)가 "쓰고",
        # _tick_relay(타이머 스레드)가 "읽습니다". 위에서 두 콜백을 서로
        # 다른 콜백 그룹에 넣어 "동시에" 돌게 만들었으므로, 보호 없이는
        # 데이터 레이스(찢어진 읽기/쓰기)가 발생합니다.
        # 그래서 접근을 threading.Lock 으로 직렬화합니다.
        # ★ Lock 이 필요한 이유 = 우리가 MultiThreadedExecutor + 분리된
        #   콜백 그룹을 선택했기 때문. (단일 스레드였다면 불필요했을 것.)
        self._lock = threading.Lock()
        self._latest_twist = None      # 아직 아무것도 못 받은 상태
        self._last_rx_time = None      # 마지막 수신 시각 (초)
        # _publish_count, _last_log_time 은 _tick_relay(타이머 콜백)에서만
        # 읽고 씁니다. 타이머 콜백은 cb_timer(MutuallyExclusiveCallbackGroup)
        # 단일 그룹이라 동시에 두 번 실행되지 않으므로, 이 두 변수는
        # 사실상 단일 스레드에서만 접근됩니다 → Lock 으로 보호할 필요가 없습니다.
        # (반대로 _latest_twist/_last_rx_time 은 두 그룹이 공유하므로 Lock 필수.)
        self._publish_count = 0        # 지금까지 재발행한 총 메시지 수 (상태 표시용)
        # 0.0 으로 시작 — time.monotonic() 은 첫 호출부터 큰 양수를 반환하므로
        # (now - 0.0) >= LOG_PERIOD_S 가 즉시 참이 되어 첫 상태 로그가 바로 찍힙니다.
        # (이는 의도된 동작: 기동 직후 한 번 상태를 보여 줌.)
        self._last_log_time = 0.0      # 마지막으로 상태 로그를 찍은 시각 (로그 throttling)

        # ── 퍼블리셔 생성 ─────────────────────────────────────────────────
        # create_publisher(메시지_타입, 토픽_이름, QoS).
        #
        # ① 메시지 타입: Twist (정확히는 geometry_msgs/msg/Twist)
        #    J100 skid-steer 로봇은 이 중 linear.x(전진/후진, m/s)와
        #    angular.z(좌/우 회전, rad/s)만 실제로 사용합니다. 나머지 필드는 0.
        # ② 토픽 이름: CMD_VEL_TOPIC = '/j100_0001/cmd_vel' (절대 경로).
        #    '/j100_0001/' 는 robot.yaml 에서 정해지는 로봇 네임스페이스.
        # ③ QoS: 위에서 정의한 qos. 발행 측도 BEST_EFFORT/VOLATILE 로 두어
        #    로봇/시뮬레이터의 cmd_vel 구독자(보통 BEST_EFFORT)와 호환되게 합니다.
        self.publisher = self.create_publisher(Twist, CMD_VEL_TOPIC, qos)

        # ── 서브스크라이버 생성 ───────────────────────────────────────────
        # create_subscription(메시지_타입, 토픽_이름, 콜백, QoS, callback_group=...).
        #
        # callback_group=self.cb_sub 가 멀티스레딩의 핵심 한 줄입니다.
        #   이 구독 콜백을 타이머 콜백과 "다른" MutuallyExclusiveCallbackGroup
        #   에 배정함으로써, MultiThreadedExecutor 아래에서 구독 콜백과
        #   타이머 콜백이 서로 다른 스레드에서 동시에 실행될 수 있습니다.
        # QoS 는 반드시 위 BEST_EFFORT qos 그대로 — Node 1 의 BEST_EFFORT
        #   퍼블리셔와 호환되어야 데이터가 실제로 들어옵니다(#1 gotcha 참고).
        self.subscription = self.create_subscription(
            Twist,
            JOY_CMD_TOPIC,
            self._on_joy_cmd,
            qos,
            callback_group=self.cb_sub,
        )

        # ── 재발행 타이머 (20 Hz) ─────────────────────────────────────────
        # rclpy 타이머는 RELAY_PERIOD_S(0.05 s = 20 Hz)마다 _tick_relay 를 호출합니다.
        #
        # callback_group=self.cb_timer 도 멀티스레딩의 핵심 한 줄입니다.
        #   타이머 콜백을 구독 콜백과 "다른" 그룹에 배정해, 입력 수신과
        #   무관하게 일정한 20Hz 출력이 별도 스레드에서 흐르도록 합니다.
        #   → 입력 레이트와 출력 레이트를 분리(decouple)하는 것이 목적.
        self.timer = self.create_timer(
            RELAY_PERIOD_S,
            self._tick_relay,
            callback_group=self.cb_timer,
        )

        # 시작 시 안전을 위해 즉시 zero Twist 를 한 번 발행합니다.
        # 이전 실행에서 로봇이 움직이고 있었던 경우를 대비합니다.
        self.publisher.publish(Twist())

        self.get_logger().info(
            f'cmd_vel_relay 시작: 구독 {JOY_CMD_TOPIC} → 재발행 {CMD_VEL_TOPIC} '
            f'@ {1.0 / RELAY_PERIOD_S:.0f} Hz (MultiThreadedExecutor, watchdog '
            f'{INPUT_TIMEOUT:.2f} s)'
        )

    # ── 구독 콜백 (cb_sub 그룹 / 구독 스레드에서 실행) ────────────────────
    def _on_joy_cmd(self, msg: Twist):
        """/joystick_cmd 에서 Twist 가 들어올 때마다 호출되는 구독 콜백.

        하는 일은 단순합니다: 받은 Twist 와 "받은 시각"을 공유 상태에
        저장만 합니다. 실제 재발행은 타이머 콜백(_tick_relay)이 담당합니다.
        이렇게 "수신"과 "발행"을 분리해야 입력 레이트와 출력 레이트가
        서로 독립적이 됩니다(릴레이 노드의 핵심 설계).

        이 콜백은 cb_sub 그룹에 속해 타이머 콜백과 다른 스레드에서
        동시에 실행될 수 있으므로, 공유 상태 갱신을 Lock 으로 보호합니다.
        """
        # 현재 시각(초). watchdog 경과시간 비교용이므로 단조 시계(monotonic)를
        # 사용합니다. ROS clock(벽시계/sim time)을 쓰면 시계 점프·sim 정지 시
        # watchdog 이 오작동할 수 있습니다(상단 INPUT_TIMEOUT 옆 설명 참고).
        now = time.monotonic()

        # ── 공유 상태 갱신은 반드시 Lock 안에서 ──────────────────────────
        # 타이머 스레드(_tick_relay)가 같은 변수를 읽는 도중에 우리가
        # 쓰면 찢어진 값을 볼 수 있으므로 with self._lock 으로 직렬화합니다.
        with self._lock:
            self._latest_twist = msg
            self._last_rx_time = now

    # ── 재발행 타이머 콜백 (cb_timer 그룹 / 타이머 스레드에서 실행) ───────
    def _tick_relay(self):
        """RELAY_PERIOD_S(0.05 s = 20 Hz)마다 호출되는 핵심 재발행 콜백.

        [이 콜백이 릴레이 노드의 존재 이유를 구현하는 곳]
          - 입력(/joystick_cmd) 주기가 불규칙해도 항상 일정한 20Hz 로
            cmd_vel 을 발행 → 출력 레이트를 입력 레이트로부터 분리(decouple).
          - watchdog: 입력이 INPUT_TIMEOUT(0.5 s) 이상 끊기면(또는 아직
            한 번도 못 받았으면) zero Twist 를 능동적으로 발행 →
            joystick_publisher 가 죽거나 멈춰도 로봇이 확실히 정지.
            "아무것도 안 보냄"이 아니라 "0 을 능동적으로 보냄"이어야
            watchdog 타임아웃 없이도 정지 상태가 유지됩니다.

        cb_timer 그룹에 속해 구독 콜백과 다른 스레드에서 동시에 돌 수
        있으므로, 공유 상태를 읽을 때 Lock 으로 보호합니다.
        """
        # _on_joy_cmd 와 '같은' 시계(time.monotonic)를 써야 두 시각의 차이가
        # 의미 있게 계산됩니다. (ROS clock 과 섞어 쓰면 안 됨 — 상단 설명 참고.)
        now = time.monotonic()

        # ── 공유 상태 읽기는 반드시 Lock 안에서 ──────────────────────────
        # 구독 스레드(_on_joy_cmd)가 같은 변수를 쓰는 도중에 읽으면
        # 찢어진 값을 볼 수 있으므로 with self._lock 으로 직렬화합니다.
        # Lock 점유 시간을 짧게 유지하려고, 값만 지역 변수로 복사한 뒤
        # 실제 판단/발행은 Lock 밖에서 합니다.
        with self._lock:
            latest = self._latest_twist
            last_rx = self._last_rx_time

        # ── watchdog 판정 ────────────────────────────────────────────────
        # 아직 한 번도 못 받았거나(last_rx is None),
        # 마지막 수신 후 INPUT_TIMEOUT 을 초과했으면 → 입력이 끊긴 것으로 간주.
        stale = (last_rx is None) or ((now - last_rx) > INPUT_TIMEOUT)

        if stale or latest is None:
            # 입력 끊김/미수신 → zero Twist 능동 발행(active stop).
            # Twist() 는 모든 필드가 0.0 으로 초기화되므로 정지 명령입니다.
            out = Twist()
            watchdog_stopped = True
        else:
            # 정상: 가장 최근에 받은 Twist 를 그대로 재발행(republish).
            out = latest
            watchdog_stopped = False

        # ROS2 미들웨어(DDS)를 통해 /j100_0001/cmd_vel 로 전송.
        # 노드가 이미 destroy 된 뒤 호출되면 예외가 날 수 있어 방어합니다.
        try:
            self.publisher.publish(out)
            self._publish_count += 1
        except Exception:
            return  # 노드가 비정상 종료 중 — 조용히 빠져나갑니다.

        # ── 상태 로그 (약 2초에 한 번만; throttling) ─────────────────────
        # 매 tick(50 ms)마다 찍으면 로그가 폭주하므로 LOG_PERIOD_S 마다만 출력.
        if (now - self._last_log_time) >= LOG_PERIOD_S:
            self._last_log_time = now
            rate_hz = 1.0 / RELAY_PERIOD_S
            # get_subscription_count(): 이 퍼블리셔를 구독 중인 노드 수.
            #   0 이면 로봇/시뮬레이터가 아직 실행 안 됐거나 네임스페이스 불일치.
            try:
                n_sub = self.publisher.get_subscription_count()
            except Exception:
                n_sub = -1

            if watchdog_stopped:
                # 입력이 끊겨 watchdog 으로 zero 를 보내는 중.
                self.get_logger().warn(
                    f'watchdog 정지: {JOY_CMD_TOPIC} 입력 끊김(>{INPUT_TIMEOUT:.2f}s) '
                    f'→ zero Twist 발행 중 @ {rate_hz:.0f} Hz '
                    f'(sub on {CMD_VEL_TOPIC}: {n_sub}, sent {self._publish_count})'
                )
            elif n_sub <= 0:
                # 정상 중계 중이지만 cmd_vel 구독자가 없음 → 로봇/시뮬레이터 미실행 의심.
                self.get_logger().warn(
                    f'재발행 중이나 {CMD_VEL_TOPIC} 구독자 없음 — '
                    f'네임스페이스/로봇 실행 여부 확인 '
                    f'(sent {self._publish_count})'
                )
            else:
                # 정상 중계 중.
                self.get_logger().info(
                    f'정상 중계: {JOY_CMD_TOPIC} → {CMD_VEL_TOPIC} '
                    f'@ {rate_hz:.0f} Hz '
                    f'({n_sub} subscriber{"s" if n_sub != 1 else ""}, '
                    f'sent {self._publish_count})'
                )

    # ── 안전 종료용: 마지막 zero Twist 발행 ──────────────────────────────
    def publish_zero(self):
        """프로세스 종료 직전에 한 번 호출 — zero Twist 를 발행해

        로봇이 마지막 이동 명령을 받은 채로 남지 않도록 합니다.
        joystick_ui.py 의 종료 시 zero 발행 패턴과 동일한 안전 장치입니다.
        """
        try:
            self.publisher.publish(Twist())
        except Exception:
            pass  # 이미 노드가 비정상 상태라면 무시


# ────────────────────────────────────────────────────────────────────────
# Entry
# ────────────────────────────────────────────────────────────────────────
def main():
    # ── 1. ROS2 초기화 ────────────────────────────────────────────────
    # rclpy.init() 은 ROS2 클라이언트 라이브러리(rclpy)를 초기화합니다.
    # 내부적으로 DDS(Data Distribution Service) 미들웨어와의 연결을 설정합니다.
    # 한 프로세스에서 반드시 한 번만 호출해야 하며, 두 번 호출하면 오류가 납니다.
    rclpy.init()

    # ── 2. 노드 생성 ──────────────────────────────────────────────────
    # CmdVelRelay() 를 인스턴스화하면 내부에서 super().__init__('cmd_vel_relay') 가
    # 호출되어 ROS2 그래프에 'cmd_vel_relay' 노드가 등록됩니다.
    # 실행 중에 `ros2 node list` 를 치면 /cmd_vel_relay 가 보입니다.
    node = CmdVelRelay()

    # ── 3. 실행기(Executor) 설정 — MultiThreadedExecutor (필수) ────────
    # Executor 는 노드의 콜백(타이머, 서브스크라이버 등)을 어떤 스레드에서
    # 실행할지 결정합니다.
    #
    # 이 노드는 SingleThreadedExecutor 를 쓰지 않습니다(요구사항상 금지).
    #   SingleThreadedExecutor 라면 구독 콜백과 타이머 콜백이 한 스레드에서
    #   순차 실행되어, 입력이 늦거나 튀면 그동안 20Hz 출력 타이머까지
    #   같이 막혀 cmd_vel 레이트가 무너집니다.
    #
    # MultiThreadedExecutor 는 여러 스레드를 두고 콜백을 병렬 실행합니다.
    #   단, 실제 병렬 실행은 콜백들이 "서로 다른 콜백 그룹" 에 있을 때만
    #   일어납니다. 이 노드는 _on_joy_cmd 를 cb_sub 에, _tick_relay 를
    #   cb_timer 에 (서로 다른 MutuallyExclusiveCallbackGroup) 배정했으므로
    #   구독 콜백과 타이머 콜백이 다른 스레드에서 동시에 돕니다.
    #
    # num_threads=4: 콜백 그룹이 2개이므로 최소 2개면 되지만, 여유 있게
    #   4개를 줍니다(향후 콜백 추가 대비). 인자를 생략하면 CPU 코어 수만큼
    #   잡히는데, 여기서는 동작을 명확히 보이려고 명시합니다.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)  # 이 executor 가 관리할 노드를 등록합니다.

    # ── 4. spin (블로킹) ──────────────────────────────────────────────
    # executor.spin() 은 콜백을 계속 처리하는 블로킹 루프입니다.
    # 이 노드는 GUI 가 없어 메인 스레드를 점유해도 무방하므로 여기서 직접
    # spin 합니다(joystick_ui.py 와 달리 별도 spin 스레드가 필요 없음).
    # Ctrl+C(SIGINT) 가 들어오면 KeyboardInterrupt 로 빠져나옵니다.
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass  # 사용자가 Ctrl+C — 정상 종료 경로
    finally:
        # ── 5. 종료 시 안전 정리 ──────────────────────────────────────
        # ROS2 를 완전히 종료하기 전에 zero Twist 를 한 번 더 발행합니다.
        # 이렇게 하면 종료 직전에 로봇이 마지막 이동 명령을 받은 채로
        # 남지 않습니다(safety zero BEFORE tearing down ROS).
        node.publish_zero()

        # executor 를 먼저 종료해 spin 루프 관련 리소스를 정리합니다.
        executor.shutdown()

        # node.destroy_node() → 노드에 연결된 ROS2 리소스(퍼블리셔,
        #   서브스크라이버, 타이머 등) 해제.
        # rclpy.try_shutdown() → DDS 미들웨어 연결 해제 및 rclpy 종료.
        # 반드시 destroy_node() 가 먼저, rclpy.try_shutdown() 이 나중입니다.
        # 순서가 바뀌면 내부 리소스가 이미 해제된 상태에서 노드를 파괴하려
        # 해 오류가 발생할 수 있습니다(joystick_ui.py 와 동일한 순서 원칙).
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

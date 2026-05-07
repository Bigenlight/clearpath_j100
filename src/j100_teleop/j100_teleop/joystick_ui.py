#!/usr/bin/env python3
"""
PyQt5 기반 가상 조이스틱 UI — geometry_msgs/msg/Twist 퍼블리셔

[이 파일이 하는 일]
  마우스로 드래그하는 2D 조이스틱 위젯을 화면에 띄우고,
  조이스틱 위치를 선속도(linear.x)·각속도(angular.z)로 변환하여
  ROS2 토픽으로 20Hz(50 ms 주기)마다 발행합니다.

[ROS2 노드 정보]
  노드 이름 : joystick_ui
              → 실행 후 `ros2 node list` 를 치면 /joystick_ui 가 보입니다.
  발행 토픽 : /j100_0001/cmd_vel
  메시지 타입: geometry_msgs/msg/Twist

[실행 방법]
  이 파일은 j100_teleop 패키지의 메인 노드입니다.
  $ source /opt/ros/humble/setup.bash        # ROS2 환경 설정 (한 번만)
  $ source install/setup.bash                # 패키지 빌드 후 오버레이 설정
  $ ros2 run j100_teleop joystick_ui         # 단독 노드 실행

[통신 대상]
  Clearpath J100(Jackal) 로봇 또는 그 시뮬레이터.
  토픽 앞의 /j100_0001/ 은 robot.yaml 에서 결정되는 네임스페이스입니다.
  실제 로봇이나 clearpath_gz 시뮬레이터가 같은 네임스페이스로
  /j100_0001/cmd_vel 을 구독해야 로봇이 움직입니다.
"""
# ── 표준 라이브러리 ────────────────────────────────────────────────────────
import math       # 유클리드 거리 계산(math.hypot) 등 수학 함수
import signal     # Ctrl+C(SIGINT) 처리를 위한 시그널 핸들러
import sys        # sys.argv — Qt 앱 초기화에 필요
import threading  # ROS spin을 별도 스레드에서 돌리기 위해 사용

# ── ROS2 클라이언트 라이브러리(rclpy) ─────────────────────────────────────
import rclpy                                    # ROS2 파이썬 바인딩의 핵심 모듈
from geometry_msgs.msg import Twist             # cmd_vel 에 쓰이는 메시지 타입
from rclpy.executors import SingleThreadedExecutor  # 단일 스레드에서 노드를 spin
from rclpy.node import Node                     # 모든 ROS2 노드의 기반 클래스
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
# QoSProfile : 퍼블리셔/서브스크라이버의 통신 품질(Quality of Service) 설정 모음

# ── PyQt5 GUI 라이브러리 ──────────────────────────────────────────────────
from PyQt5.QtCore import Qt, QEvent, QPointF, QTimer, pyqtSignal
from PyQt5.QtGui import QBrush, QColor, QKeySequence, QPainter, QPen, QRadialGradient
from PyQt5.QtWidgets import (
    QApplication, QHBoxLayout, QLabel, QMainWindow, QPushButton,
    QShortcut, QSlider, QVBoxLayout, QWidget,
)


# ── 전역 상수 ─────────────────────────────────────────────────────────────

# 발행할 ROS2 토픽 이름.
#   앞에 '/'가 붙으면 절대 경로이므로 어떤 네임스페이스에서 실행해도
#   항상 이 이름 그대로 발행됩니다.
#   '/j100_0001/' 부분은 Clearpath J100 로봇의 네임스페이스(robot_id)로,
#   clearpath_gz 시뮬레이터의 robot.yaml 파일에서 설정됩니다.
CMD_VEL_TOPIC = '/j100_0001/cmd_vel'

# 발행 주기: 50 ms = 20 Hz.
#   cmd_vel 을 일정 시간 못 받으면 로봇이 안전을 위해 자동 정지하는
#   watchdog(twist_mux timeout 기본값 0.5 s) 이 있으므로,
#   그보다 훨씬 자주(20 Hz) 보내야 로봇이 계속 움직입니다.
#   일반적으로 cmd_vel 은 10 ~ 50 Hz 범위가 적합합니다.
#   너무 느리면 watchdog 에 걸려 정지, 너무 빠르면 CPU/네트워크 낭비.
PUBLISH_PERIOD_MS = 50          # 20 Hz, well under twist_mux 0.5 s timeout

DEFAULT_MAX_LINEAR = 0.5        # 슬라이더 초기값: 선속도 최대치 (m/s)
DEFAULT_MAX_ANGULAR = 1.0       # 슬라이더 초기값: 각속도 최대치 (rad/s)

# 슬라이더가 가질 수 있는 하드 상한.
#   실제 컨트롤러(robot.yaml)에서 linear 2.0 m/s, angular 4.0 rad/s 로
#   추가 클램핑이 되지만, UI 에서는 그보다 넉넉하게 설정해 둡니다.
HARD_MAX_LINEAR = 4.0           # slider upper bound (controller clamps at 2.0 by robot.yaml)
HARD_MAX_ANGULAR = 8.0          # slider upper bound (controller clamps at 4.0 by robot.yaml)

# 조이스틱 축 스냅 임계값.
#   노브가 가로/세로 축으로부터 15% 이내에 있을 때 해당 축에 딱 맞게 보정.
#   예: 거의 정면으로 밀었는데 약간 비뚤어진 경우 → 순수 전진으로 교정.
AXIS_SNAP_FRAC = 0.15           # snap to pure axis when within 15% of cardinal


# ────────────────────────────────────────────────────────────────────────
# Joystick widget
# ────────────────────────────────────────────────────────────────────────
class JoystickWidget(QWidget):
    """마우스로 드래그하는 스프링 복귀형 2D 조이스틱 위젯.

    손을 떼면 노브가 중앙으로 돌아오며(스프링 복귀),
    정규화된 좌표 (x_right, y_up) ∈ [-1, 1] 을 position_changed 시그널로 방출합니다.
    즉, 오른쪽으로 밀면 x=+1, 위로 밀면 y=+1 입니다.
    """

    # pyqtSignal: Qt의 시그널-슬롯 메커니즘.
    # 노브 위치가 바뀔 때마다 (float x, float y) 두 개의 실수값을 방출합니다.
    # MainWindow._on_joystick 이 이 시그널을 받아서 Twist 값을 갱신합니다.
    position_changed = pyqtSignal(float, float)

    def __init__(self, parent=None, size=300):
        super().__init__(parent)
        self.setFixedSize(size, size)
        self._outer_radius = size * 0.45         # 바깥 원의 반지름 (픽셀)
        self._knob_radius = self._outer_radius * 0.25  # 노브(손잡이) 반지름 (픽셀)
        self._max_offset = self._outer_radius - self._knob_radius  # 노브가 이동할 수 있는 최대 거리
        self._knob_pos = QPointF(0.0, 0.0)   # offset from center, Qt coords (y-down)
        self._dragging = False                   # 현재 마우스를 클릭·드래그 중인지 여부

    def _center(self) -> QPointF:
        # 위젯 중앙 좌표를 반환합니다. 크기가 바뀌어도 항상 정확한 중앙을 계산합니다.
        return QPointF(self.width() / 2.0, self.height() / 2.0)

    def paintEvent(self, _event):
        # Qt가 위젯을 다시 그려야 할 때마다 자동으로 호출하는 메서드입니다.
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)  # 가장자리 계단 현상(aliasing) 제거
        c = self._center()

        # outer dish — 바깥 원형 배경
        grad = QRadialGradient(c, self._outer_radius)
        grad.setColorAt(0.0, QColor(60, 65, 75))
        grad.setColorAt(1.0, QColor(30, 32, 38))
        p.setBrush(QBrush(grad))
        p.setPen(QPen(QColor(20, 22, 26), 2))
        p.drawEllipse(c, self._outer_radius, self._outer_radius)

        # crosshair — 십자선: 조이스틱 축의 방향을 시각적으로 표시
        p.setPen(QPen(QColor(110, 115, 125), 1, Qt.DashLine))
        p.drawLine(QPointF(c.x() - self._max_offset, c.y()),
                   QPointF(c.x() + self._max_offset, c.y()))
        p.drawLine(QPointF(c.x(), c.y() - self._max_offset),
                   QPointF(c.x(), c.y() + self._max_offset))
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(140, 145, 155))
        p.drawEllipse(c, 3, 3)  # 중앙 점

        # knob — 실제로 드래그하는 원형 노브
        kpos = c + self._knob_pos
        # 드래그 중이면 파란색, 아니면 회색으로 표시하여 상태를 직관적으로 구분
        knob_color = QColor(90, 170, 240) if self._dragging else QColor(180, 190, 205)
        edge_color = QColor(40, 90, 140) if self._dragging else QColor(80, 90, 105)
        kgrad = QRadialGradient(kpos, self._knob_radius)
        kgrad.setColorAt(0.0, knob_color.lighter(120))
        kgrad.setColorAt(1.0, knob_color)
        p.setBrush(QBrush(kgrad))
        p.setPen(QPen(edge_color, 2))
        p.drawEllipse(kpos, self._knob_radius, self._knob_radius)

    def _set_knob_from_mouse(self, pos):
        """마우스 위치(pos)를 받아 노브를 그 방향으로 이동하고 시그널을 방출합니다."""
        c = self._center()
        dx, dy = pos.x() - c.x(), pos.y() - c.y()
        dist = math.hypot(dx, dy)  # 중앙에서 마우스까지의 유클리드 거리 (픽셀)

        # 노브가 바깥 원을 벗어나지 못하도록 클램핑
        if dist > self._max_offset:
            dx = dx * self._max_offset / dist
            dy = dy * self._max_offset / dist

        # axis snap: pure straight / pure turn made easy
        # 축 스냅: 거의 가로/세로 방향일 때 해당 축에 딱 맞춰 줌
        # 예: x 이동량이 전체의 15% 미만이면 x=0 으로 보정 → 순수 직진/회전
        snap_thresh = self._max_offset * AXIS_SNAP_FRAC
        if abs(dx) < snap_thresh:
            dx = 0.0
        if abs(dy) < snap_thresh:
            dy = 0.0

        self._knob_pos = QPointF(dx, dy)
        self.update()  # 화면 다시 그리기 요청 (paintEvent 를 Qt 가 곧 호출)

        # ── 슬라이더 → Twist 매핑을 위한 정규화 ───────────────────────────
        # dx, dy 는 픽셀 단위이므로 _max_offset 으로 나눠 [-1, 1] 범위로 정규화합니다.
        nx = dx / self._max_offset   # 오른쪽(+) ↔ 왼쪽(-): angular.z 에 대응
        ny = dy / self._max_offset   # Qt y 축은 아래가 +, 위가 -

        # ny 에 -1 을 곱해 "위로 밀면 y_up = +1" 이 되도록 반전합니다.
        # 이 y_up 값이 나중에 Twist.linear.x (전진 속도, m/s) 로 변환됩니다.
        self.position_changed.emit(nx, -ny)   # flip y so up = +1

    def mousePressEvent(self, e):
        # 마우스 왼쪽 버튼을 누르면 드래그 시작, 노브를 그 위치로 이동
        if e.button() == Qt.LeftButton:
            self._dragging = True
            self._set_knob_from_mouse(e.pos())

    def mouseMoveEvent(self, e):
        # 드래그 중(버튼 누른 채 이동)에만 노브 위치 갱신
        if self._dragging:
            self._set_knob_from_mouse(e.pos())

    def mouseReleaseEvent(self, e):
        # 마우스 버튼을 놓으면: 노브를 중앙(0, 0)으로 되돌리고 정지 명령(0.0, 0.0) 방출
        # → 로봇이 즉시 멈춥니다. 스프링 복귀 효과.
        if e.button() == Qt.LeftButton and self._dragging:
            self._dragging = False
            self._knob_pos = QPointF(0.0, 0.0)
            self.update()
            self.position_changed.emit(0.0, 0.0)

    def force_zero(self):
        """노브를 즉시 중앙으로 복귀시키고 (0.0, 0.0) 시그널을 방출합니다.

        앱이 포커스를 잃거나 E-STOP 이 활성화될 때 호출됩니다.
        이미 중앙이면 불필요한 갱신을 하지 않도록 조건 검사를 합니다.
        """
        if self._dragging or self._knob_pos != QPointF(0.0, 0.0):
            self._dragging = False
            self._knob_pos = QPointF(0.0, 0.0)
            self.update()
            self.position_changed.emit(0.0, 0.0)


# ────────────────────────────────────────────────────────────────────────
# ROS2 node
# ────────────────────────────────────────────────────────────────────────
class TeleopNode(Node):
    """cmd_vel 퍼블리셔를 담당하는 ROS2 노드 클래스.

    rclpy.node.Node 를 상속받아 ROS2 노드로서의 기능(퍼블리셔 생성,
    타이머, 로깅 등)을 모두 사용할 수 있습니다.
    """

    def __init__(self):
        # ── ROS2 노드 초기화 ──────────────────────────────────────────────
        # super().__init__('joystick_ui') 는 부모 클래스인 Node 의 __init__ 을 호출합니다.
        # 인자로 전달한 문자열 'joystick_ui' 가 이 노드의 ROS2 그래프상 이름이 됩니다.
        # 실행 후 터미널에서 `ros2 node list` 를 입력하면 /joystick_ui 로 나타납니다.
        # 노드 이름은 ROS2 그래프에서 고유해야 하며, 같은 이름의 노드가 두 개 실행되면
        # 충돌이 발생할 수 있습니다.
        super().__init__('joystick_ui')

        # ── QoS(통신 품질) 설정 ──────────────────────────────────────────
        # QoSProfile 은 퍼블리셔와 서브스크라이버가 메시지를 어떻게 주고받을지 결정합니다.
        # cmd_vel 처럼 "최신 값만 중요하고 과거 메시지는 필요 없는" 토픽에는 아래 설정이 적합합니다.
        qos = QoSProfile(
            # BEST_EFFORT: 전송 실패 시 재전송하지 않음. 실시간성을 우선시합니다.
            # (반대: RELIABLE — 재전송 보장, 지연 가능성 있음)
            reliability=ReliabilityPolicy.BEST_EFFORT,
            # VOLATILE: 새로 연결한 서브스크라이버에게 과거 메시지를 보내지 않음.
            durability=DurabilityPolicy.VOLATILE,
            # KEEP_LAST + depth=10: 최대 10개의 메시지만 큐에 보관합니다.
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            # depth(큐 크기)=10: 서브스크라이버가 처리 속도가 느릴 때 최대 10개까지
            # 메시지를 버퍼에 쌓아 둡니다. cmd_vel 처럼 빠른 토픽은 10이면 충분합니다.
        )

        # ── 퍼블리셔 생성 ─────────────────────────────────────────────────
        # create_publisher(메시지_타입, 토픽_이름, QoS) 형태로 호출합니다.
        #
        # ① 첫 번째 인자 — 메시지 타입: Twist
        #    정확한 타입명은 geometry_msgs/msg/Twist 입니다.
        #    이 타입은 아래와 같은 구조를 가집니다 (단위: SI):
        #
        #    geometry_msgs/msg/Twist 메시지 구조:
        #      Vector3 linear      # 선속도 (m/s)
        #        float64 x         # 전진(+) / 후진(-). J100 skid-steer 에서 주요 축.
        #        float64 y         # 좌/우 평행이동. J100 은 skid-steer 이므로 사용 안 함(0 고정).
        #        float64 z         # 위/아래. 지상 로봇에서는 사용 안 함(0 고정).
        #      Vector3 angular     # 각속도 (rad/s)
        #        float64 x         # 롤(roll). 지상 로봇에서는 사용 안 함(0 고정).
        #        float64 y         # 피치(pitch). 지상 로봇에서는 사용 안 함(0 고정).
        #        float64 z         # 요(yaw). 좌회전(+) / 우회전(-). J100 에서 핵심 필드!
        #
        #    이 UI의 슬라이더 상한 (코드 상수 HARD_MAX_LINEAR / HARD_MAX_ANGULAR):
        #      linear.x  : ±4.0 m/s
        #      angular.z : ±8.0 rad/s
        #    실제 J100 컨트롤러는 robot.yaml 의 velocity_limits 로 추가 클램핑되며,
        #    실 하드웨어는 그보다 더 낮은 물리 한계(예: ±2.0 m/s)를 가질 수 있음.
        #
        # ② 두 번째 인자 — 토픽 이름: CMD_VEL_TOPIC = '/j100_0001/cmd_vel'
        #    앞에 '/' 가 붙으면 절대 경로 토픽이므로, 이 노드가 어느 네임스페이스에서
        #    실행되더라도 항상 '/j100_0001/cmd_vel' 로 발행됩니다.
        #    '/j100_0001/' 부분은 Clearpath J100 로봇의 네임스페이스(robot_id)로,
        #    clearpath_gz 시뮬레이터/실 로봇의 robot.yaml 에서 설정됩니다.
        #    확인: `ros2 topic list | grep cmd_vel`
        #
        # ③ 세 번째 인자 — QoS 설정 객체 (위에서 정의한 qos 변수)
        #    정수(depth만)도 가능하지만, 여기서는 QoSProfile 로 상세 설정합니다.
        self.publisher = self.create_publisher(Twist, CMD_VEL_TOPIC, qos)

    def publish_twist(self, linear_x: float, angular_z: float) -> bool:
        """Twist 메시지를 만들어 cmd_vel 토픽으로 발행합니다.

        Args:
            linear_x:  전진 속도 (m/s). + 는 전진, - 는 후진.
                       슬라이더 상한(코드 상수 HARD_MAX_LINEAR): ±4.0 m/s.
                       실 하드웨어는 robot.yaml velocity_limits 및 물리 한계로 추가 클램핑됨.
            angular_z: 회전 각속도 (rad/s). + 는 좌회전(반시계), - 는 우회전(시계).
                       REP-103(ROS 표준 좌표계) 기준: z축 + 방향이 반시계 회전.
                       슬라이더 상한(코드 상수 HARD_MAX_ANGULAR): ±8.0 rad/s.
                       실 하드웨어는 robot.yaml velocity_limits 및 물리 한계로 추가 클램핑됨.

        Returns:
            True  — 발행 성공
            False — 예외 발생 (노드 종료 후 등)
        """
        # geometry_msgs/msg/Twist 메시지 인스턴스 생성.
        # 명시적으로 값을 넣지 않은 필드(linear.y, linear.z, angular.x, angular.y)는
        # ROS2 에서 자동으로 0.0 으로 초기화됩니다.
        # J100 skid-steer 로봇은 linear.x 와 angular.z 만 실제로 사용합니다.
        msg = Twist()
        msg.linear.x = float(linear_x)   # 전진/후진 선속도 (m/s)
        msg.angular.z = float(angular_z)  # 좌/우 회전 각속도 (rad/s)
        try:
            self.publisher.publish(msg)  # ROS2 미들웨어(DDS)를 통해 전송
            return True
        except Exception:
            # 노드가 이미 destroy_node() 된 이후에 publish 를 시도하면
            # 예외가 발생할 수 있습니다. 안전하게 False 를 반환합니다.
            return False

    def subscriber_count(self) -> int:
        """이 퍼블리셔를 구독 중인 서브스크라이버의 수를 반환합니다.

        0 이면 로봇(또는 시뮬레이터)이 아직 실행되지 않았거나
        네임스페이스가 맞지 않는다는 의미입니다.
        -1 은 조회 자체가 실패했을 때 반환합니다.
        """
        try:
            return self.publisher.get_subscription_count()
        except Exception:
            return -1


# ────────────────────────────────────────────────────────────────────────
# Main window
# ────────────────────────────────────────────────────────────────────────
class MainWindow(QMainWindow):
    """조이스틱 UI 의 메인 창 클래스.

    TeleopNode(ROS2 노드)를 받아서 GUI 위젯들을 배치하고,
    QTimer 를 통해 주기적으로 cmd_vel 을 발행합니다.

    [Qt 스레드와 ROS spin 의 관계]
      이 클래스는 Qt 메인 스레드(GUI 스레드)에서 실행됩니다.
      ROS2 의 executor.spin() 은 별도의 백그라운드 스레드(spin_thread)에서 실행됩니다.
      두 스레드가 동시에 self.node 에 접근할 수 있으므로, publish_twist() 는
      thread-safe 하게 설계되어 있어야 합니다(현재 구현은 단순 publish 이므로 안전).

    [QTimer vs rclpy Timer]
      여기서는 rclpy.create_timer() 대신 Qt 의 QTimer 를 사용합니다.
      QTimer 는 Qt 이벤트 루프에서 실행되므로 GUI 위젯(QLabel 등)을 직접 업데이트할 수 있습니다.
      rclpy 타이머는 ROS spin 스레드에서 실행되므로 GUI 를 직접 건드리면 크래시가 납니다.
      → 따라서 "발행 + UI 갱신" 을 QTimer 콜백(_tick_publish)에서 한 번에 처리합니다.
    """

    def __init__(self, node: TeleopNode):
        super().__init__()
        self.node = node  # ROS2 노드 참조 보관 (발행에 사용)
        self.setWindowTitle('J100 Joystick Control')

        # ── 내부 상태 변수 ─────────────────────────────────────────────────
        self._joy_x = 0.0          # 조이스틱 X 정규화 값 [-1, 1], 오른쪽이 +
        self._joy_y = 0.0          # 조이스틱 Y 정규화 값 [-1, 1], 위가 +
        self._max_linear = DEFAULT_MAX_LINEAR    # 현재 선속도 최대치 (m/s)
        self._max_angular = DEFAULT_MAX_ANGULAR  # 현재 각속도 최대치 (rad/s)
        self._estopped = False       # E-STOP 버튼 활성화 여부
        self._publish_count = 0      # 지금까지 발행한 총 메시지 수 (상태 표시용)
        self._last_publish_ok = True # 직전 publish 성공 여부

        # ── E-STOP (top, prominent) ─────────────────────────────────────
        # 비상 정지 버튼. setCheckable(True) 로 토글 버튼으로 만들어,
        # 한 번 누르면 E-STOP 활성, 다시 누르면 해제됩니다.
        self.btn_estop = QPushButton('EMERGENCY STOP   (Space)')
        self.btn_estop.setMinimumHeight(60)
        self.btn_estop.setCheckable(True)
        self.btn_estop.toggled.connect(self._on_estop)
        self._apply_estop_style(False)

        # 스페이스바 단축키: 마우스 없이도 빠르게 E-STOP 을 누를 수 있게 합니다.
        self._shortcut_estop = QShortcut(QKeySequence(Qt.Key_Space), self)
        self._shortcut_estop.activated.connect(lambda: self.btn_estop.toggle())

        # ── Joystick + readouts ─────────────────────────────────────────
        self.joystick = JoystickWidget(size=320)
        self.joystick.position_changed.connect(self._on_joystick)

        self.lbl_linear = QLabel('+0.000  m/s')
        self.lbl_angular = QLabel('+0.000  rad/s')
        big = self.lbl_linear.font()
        big.setFamily('Monospace'); big.setPointSize(22); big.setBold(True)
        self.lbl_linear.setFont(big); self.lbl_angular.setFont(big)
        self.lbl_linear.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.lbl_angular.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        cap_linear = QLabel('linear.x'); cap_angular = QLabel('angular.z')
        cap_font = cap_linear.font(); cap_font.setPointSize(10)
        for cap in (cap_linear, cap_angular):
            cap.setFont(cap_font); cap.setStyleSheet('color:#888;')

        readout = QVBoxLayout()
        readout.addWidget(QLabel('<b>cmd_vel (live)</b>'))
        readout.addSpacing(8)
        readout.addWidget(cap_linear)
        readout.addWidget(self.lbl_linear)
        readout.addSpacing(12)
        readout.addWidget(cap_angular)
        readout.addWidget(self.lbl_angular)
        readout.addStretch(1)
        hint = QLabel(
            "<i>Release mouse or leave window → robot stops.<br>"
            "Press <b>Space</b> for emergency stop.</i>"
        )
        hint.setStyleSheet('color:#777;')
        hint.setWordWrap(True)
        readout.addWidget(hint)

        joy_row = QHBoxLayout()
        joy_row.addWidget(self.joystick)
        joy_row.addLayout(readout, 1)

        # ── Sliders with min/max labels ────────────────────────────────
        self.lbl_max_linear = QLabel(f'Max linear:  {self._max_linear:.2f} m/s')
        self.slider_linear = self._make_slider(HARD_MAX_LINEAR, self._max_linear)
        self.slider_linear.valueChanged.connect(self._on_max_linear)
        slider_lin_row = self._slider_row(self.slider_linear, '0.10', f'{HARD_MAX_LINEAR:.2f}')

        self.lbl_max_angular = QLabel(f'Max angular: {self._max_angular:.2f} rad/s')
        self.slider_angular = self._make_slider(HARD_MAX_ANGULAR, self._max_angular)
        self.slider_angular.valueChanged.connect(self._on_max_angular)
        slider_ang_row = self._slider_row(self.slider_angular, '0.10', f'{HARD_MAX_ANGULAR:.2f}')

        # ── Status (health: subscriber count) ─────────────────────────
        self.lbl_status = QLabel('●  initializing…')
        sf = self.lbl_status.font(); sf.setPointSize(10); self.lbl_status.setFont(sf)

        # ── Root layout: e-stop on top ────────────────────────────────
        root = QVBoxLayout()
        root.addWidget(self.btn_estop)
        root.addSpacing(6)
        root.addLayout(joy_row)
        root.addSpacing(10)
        root.addWidget(self.lbl_max_linear)
        root.addLayout(slider_lin_row)
        root.addSpacing(4)
        root.addWidget(self.lbl_max_angular)
        root.addLayout(slider_ang_row)
        root.addSpacing(8)
        root.addWidget(self.lbl_status)

        container = QWidget()
        container.setLayout(root)
        container.setContentsMargins(12, 12, 12, 12)
        self.setCentralWidget(container)

        # ── 발행 루프: Qt 타이머 (QTimer) ────────────────────────────────
        # QTimer 는 Qt 이벤트 루프 안에서 일정 주기로 콜백을 호출합니다.
        # PUBLISH_PERIOD_MS = 50 ms → 1000 / 50 = 20 Hz 주기로 _tick_publish 를 실행합니다.
        #
        # 왜 rclpy 타이머 대신 QTimer 를 사용하나?
        #   rclpy.create_timer() 의 콜백은 ROS spin 스레드에서 실행됩니다.
        #   반면 QLabel.setText() 같은 Qt GUI 업데이트는 반드시 Qt 메인 스레드에서만 해야 합니다.
        #   QTimer 는 Qt 메인 스레드에서 실행되므로, 발행과 UI 갱신을 동시에 안전하게 처리할 수 있습니다.
        #
        # watchdog 관점:
        #   로봇의 twist_mux 나 controller_manager 는 일정 시간(보통 0.5 s) 동안
        #   cmd_vel 을 받지 못하면 로봇을 자동으로 정지시킵니다(watchdog timeout).
        #   20 Hz = 50 ms 주기로 발행하면 이 타임아웃보다 10배 빠르므로 안전합니다.
        self.publish_timer = QTimer(self)
        self.publish_timer.timeout.connect(self._tick_publish)
        self.publish_timer.start(PUBLISH_PERIOD_MS)

        # 시작 시 안전을 위해 즉시 zero Twist 를 한 번 발행합니다.
        # 이전 실행에서 로봇이 움직이고 있었던 경우를 대비합니다.
        self.node.publish_twist(0.0, 0.0)

    # ── helpers ───────────────────────────────────────────────────────
    def _make_slider(self, hard_max: float, init: float) -> QSlider:
        s = QSlider(Qt.Horizontal)
        s.setRange(10, int(hard_max * 100))   # units of 0.01
        s.setValue(int(init * 100))
        s.setTickPosition(QSlider.TicksBelow)
        s.setTickInterval(int(hard_max * 25))  # ~4 ticks
        return s

    @staticmethod
    def _slider_row(slider: QSlider, lo: str, hi: str) -> QHBoxLayout:
        row = QHBoxLayout()
        l = QLabel(lo); r = QLabel(hi)
        for x in (l, r):
            f = x.font(); f.setPointSize(8); x.setFont(f)
            x.setStyleSheet('color:#888;')
        row.addWidget(l); row.addWidget(slider, 1); row.addWidget(r)
        return row

    def _apply_estop_style(self, engaged: bool):
        if engaged:
            css = ('background:#5a1a14; color:#ffe680; border:3px solid #ffd633;'
                   ' font-weight:bold; font-size:16px; padding:10px; border-radius:8px;')
        else:
            css = ('background:#e74c3c; color:white; border:none;'
                   ' font-weight:bold; font-size:18px; padding:10px; border-radius:8px;')
        self.btn_estop.setStyleSheet(f'QPushButton {{ {css} }}'
                                     f'QPushButton:pressed {{ background:#922b21; }}')

    # ── slots (시그널 수신 콜백) ──────────────────────────────────────────
    def _on_joystick(self, x: float, y: float):
        # JoystickWidget.position_changed 시그널이 방출될 때마다 호출됩니다.
        # x, y 는 [-1, 1] 범위의 정규화된 값입니다.
        # 실제 속도 변환은 _tick_publish 에서 _max_linear / _max_angular 를 곱해 이뤄집니다.
        self._joy_x = x
        self._joy_y = y

    def _on_max_linear(self, v: int):
        # 슬라이더는 정수(0.01 단위)로 값을 반환하므로 100으로 나눠 m/s 단위로 변환합니다.
        # 예: 슬라이더 값 50 → 0.50 m/s
        self._max_linear = v / 100.0
        self.lbl_max_linear.setText(f'Max linear:  {self._max_linear:.2f} m/s')

    def _on_max_angular(self, v: int):
        # 슬라이더 값을 100으로 나눠 rad/s 단위로 변환합니다.
        # 예: 슬라이더 값 100 → 1.00 rad/s
        self._max_angular = v / 100.0
        self.lbl_max_angular.setText(f'Max angular: {self._max_angular:.2f} rad/s')

    def _on_estop(self, on: bool):
        # E-STOP 버튼의 토글 상태가 바뀔 때 호출됩니다.
        # on=True → E-STOP 활성화: 조이스틱을 강제로 0으로 만들고 버튼 스타일 변경.
        # on=False → E-STOP 해제: 로봇이 다시 조이스틱 입력을 받을 준비가 됩니다.
        self._estopped = on
        self._apply_estop_style(on)
        if on:
            self.joystick.force_zero()  # 노브를 중앙으로 복귀, (0.0, 0.0) 시그널 방출
            self.btn_estop.setText('E-STOP ENGAGED — click or press Space to release')
        else:
            self.btn_estop.setText('EMERGENCY STOP   (Space)')

    # ── publish loop ──────────────────────────────────────────────────
    def _tick_publish(self):
        """QTimer 가 PUBLISH_PERIOD_MS(50 ms = 20 Hz)마다 호출하는 핵심 발행 콜백.

        조이스틱 정규화 값 → 실제 속도(m/s, rad/s) 변환 후 Twist 발행.
        """
        if self._estopped:
            # E-STOP 중에는 zero Twist 를 계속 발행합니다.
            # "아무것도 안 보내는" 것이 아니라 0 을 능동적으로 보내야
            # watchdog 타임아웃 없이 로봇이 정지 상태를 유지합니다.
            lin, ang = 0.0, 0.0
        else:
            # ── 조이스틱 정규화 값 → SI 단위 속도 변환 ─────────────────
            # _joy_y : 위(+1) ↔ 아래(-1) → Twist.linear.x (전진/후진, m/s)
            #   조이스틱을 위로 밀수록 전진 속도가 커집니다.
            #   _max_linear(슬라이더)을 곱해 실제 m/s 값으로 스케일링합니다.
            lin = self._joy_y * self._max_linear     # joystick up = forward

            # _joy_x : 오른쪽(+1) ↔ 왼쪽(-1) → Twist.angular.z (요 각속도, rad/s)
            #   REP-103(ROS 표준 좌표계): z축 +방향 = 반시계(좌회전).
            #   조이스틱 오른쪽 = 우회전 = -angular.z, 이므로 부호를 반전(-_joy_x).
            ang = -self._joy_x * self._max_angular   # joystick right = clockwise = -yaw (REP-103)

        # 계산된 선속도(lin, m/s) 와 각속도(ang, rad/s) 로 Twist 를 발행합니다.
        self._last_publish_ok = self.node.publish_twist(lin, ang)
        self._publish_count += 1

        # 화면 숫자 갱신: {lin:+.3f} 는 부호 포함 소수점 3자리 형식 (예: +0.500)
        self.lbl_linear.setText(f'{lin:+.3f}  m/s')
        self.lbl_angular.setText(f'{ang:+.3f}  rad/s')

        # health: subscriber count — 구독자 수로 연결 상태를 진단합니다.
        # 0 이면 로봇/시뮬레이터가 실행 안 됐거나 토픽 이름이 맞지 않는 것입니다.
        n_sub = self.node.subscriber_count()
        rate_hz = 1000.0 / PUBLISH_PERIOD_MS
        if self._estopped:
            color, text = '#e67e22', f'■  E-STOP ENGAGED  (publishing zeros @ {rate_hz:.0f} Hz)'
        elif not self._last_publish_ok:
            color, text = '#e74c3c', '●  publish FAILED — node may be down'
        elif n_sub <= 0:
            color, text = '#f39c12', (
                f'●  no subscribers on {CMD_VEL_TOPIC}  — '
                f'check namespace / robot is running'
            )
        else:
            color, text = '#27ae60', (
                f'●  publishing @ {rate_hz:.0f} Hz to {CMD_VEL_TOPIC}  '
                f'({n_sub} subscriber{"s" if n_sub != 1 else ""}, {self._publish_count} sent)'
            )
        self.lbl_status.setText(text)
        self.lbl_status.setStyleSheet(f'color:{color};')

    # ── safety hooks (안전 장치 이벤트 핸들러) ───────────────────────────
    def changeEvent(self, e):
        # Zero only on app-level deactivate (alt-tab, lose focus to other app).
        # NOT on focus changes between widgets in the same window (slider clicks).
        # 앱 전체가 비활성화될 때(alt-tab, 다른 앱으로 전환)만 조이스틱을 0으로 초기화합니다.
        # 같은 창 내에서 슬라이더를 클릭하는 등의 위젯 포커스 변경에는 반응하지 않습니다.
        if e.type() == QEvent.ApplicationStateChange:
            if QApplication.applicationState() != Qt.ApplicationActive:
                self.joystick.force_zero()
        super().changeEvent(e)

    def closeEvent(self, e):
        """창을 닫을 때 안전하게 정리하는 핸들러.

        [종료 순서가 중요한 이유]
          1. publish_timer.stop() → QTimer 를 먼저 멈춰 추가 발행을 차단합니다.
          2. node.publish_twist(0.0, 0.0) → ROS2 가 완전히 종료되기 전에
             zero Twist 를 한 번 더 발행합니다. 이렇게 하면 프로그램 종료 직전에
             로봇이 이동 명령을 마지막으로 받은 채로 남지 않습니다.
          3. QApplication.quit() → Qt 이벤트 루프 종료.
          4. main() 의 finally 블록에서 executor.shutdown() → spin_thread.join()
             → node.destroy_node() → rclpy.try_shutdown() 순으로 정리됩니다.
             destroy_node() 를 먼저 하고 rclpy.shutdown() 을 나중에 해야
             ROS2 내부 리소스가 올바르게 해제됩니다.
        """
        self.publish_timer.stop()
        self.node.publish_twist(0.0, 0.0)  # 종료 직전 zero Twist 발행 — 안전한 패턴
        QApplication.instance().quit()
        super().closeEvent(e)


# ────────────────────────────────────────────────────────────────────────
# Entry
# ────────────────────────────────────────────────────────────────────────
def main():
    # ── 1. ROS2 초기화 ────────────────────────────────────────────────
    # rclpy.init() 은 ROS2 클라이언트 라이브러리(rclpy)를 초기화합니다.
    # 내부적으로 DDS(Data Distribution Service) 미들웨어와의 연결을 설정합니다.
    # 한 프로세스에서 반드시 한 번만 호출해야 하며, 두 번 호출하면 오류가 납니다.
    # (인자로 sys.argv 를 넘겨도 되지만, 여기서는 기본값 사용)
    rclpy.init()

    # ── 2. 노드 생성 ──────────────────────────────────────────────────
    # TeleopNode() 를 인스턴스화하면 내부에서 super().__init__('joystick_ui') 가 호출되어
    # ROS2 그래프에 'joystick_ui' 라는 이름의 노드가 등록됩니다.
    # 실행 중에 `ros2 node list` 를 치면 /joystick_ui 가 보입니다.
    node = TeleopNode()

    # ── 3. 실행기(Executor) 설정 ──────────────────────────────────────
    # Executor 는 노드의 콜백(타이머, 서브스크라이버 등)을 어떤 스레드에서 실행할지 결정합니다.
    #
    # SingleThreadedExecutor: 모든 콜백을 단일 스레드에서 순차 실행합니다.
    #   → 이 앱에서는 ROS 콜백이 퍼블리셔뿐이므로 단일 스레드로 충분합니다.
    #
    # (참고) MultiThreadedExecutor: 콜백을 여러 스레드에서 병렬 실행합니다.
    #   → 여러 서브스크라이버/서비스가 동시에 응답해야 할 때 유리합니다.
    #   → 그러나 공유 데이터에 대한 뮤텍스(mutex) 처리가 필요해집니다.
    executor = SingleThreadedExecutor()
    executor.add_node(node)  # 이 executor 가 관리할 노드를 등록합니다.

    # ── 4. ROS spin 을 별도 스레드에서 실행 ──────────────────────────
    # executor.spin() 은 블로킹(blocking) 함수입니다. 메인 스레드에서 호출하면
    # Qt 이벤트 루프를 실행할 수 없으므로, 별도의 백그라운드 스레드에서 실행합니다.
    #
    # daemon=True: 메인 스레드(Qt 루프)가 종료되면 이 스레드도 자동으로 종료됩니다.
    #   → 프로그램 종료 시 spin_thread 가 무한 대기 상태로 남아 프로세스가 안 끝나는
    #     상황을 방지합니다.
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # ── 5. Qt 애플리케이션 초기화 ─────────────────────────────────────
    # QApplication 은 Qt 이벤트 루프와 모든 위젯의 생명주기를 관리합니다.
    # sys.argv 를 넘겨야 Qt 자체 명령줄 인자(-style 등)를 처리할 수 있습니다.
    app = QApplication(sys.argv)

    # Ctrl+C(SIGINT) 를 받으면 Qt 앱을 종료합니다.
    # 파이썬 signal 핸들러는 Qt 이벤트 루프 중에는 실행되지 않을 수 있으므로,
    # sigint_kicker 타이머(100 ms)로 파이썬 인터프리터를 주기적으로 깨워
    # 시그널 핸들러가 실제로 실행될 기회를 줍니다.
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    sigint_kicker = QTimer(app)            # parented so it isn't GC'd
    sigint_kicker.start(100)
    sigint_kicker.timeout.connect(lambda: None)  # 아무 것도 안 하는 더미 콜백 — 파이썬 wake-up용

    # ── 6. 메인 창 생성 및 표시 ──────────────────────────────────────
    window = MainWindow(node)
    window.show()

    # ── 7. Qt 이벤트 루프 실행 (블로킹) ──────────────────────────────
    # app.exec_() 는 창이 닫힐 때까지 여기서 블로킹됩니다.
    try:
        app.exec_()
    finally:
        # ── 8. 종료 시 안전 정리 ──────────────────────────────────────
        # Qt 루프가 끝난 직후, ROS2 를 완전히 종료하기 전에 zero Twist 를 발행합니다.
        # 이렇게 하면 창 X 버튼이 아닌 다른 경로(터미널 kill 등)로 종료될 때도 안전합니다.
        # safety zero BEFORE tearing down ROS
        try:
            node.publish_twist(0.0, 0.0)
        except Exception:
            pass  # 이미 노드가 비정상 상태라면 무시

        # executor 를 먼저 종료해 spin 루프를 빠져나오게 합니다.
        executor.shutdown()
        # spin_thread 가 완전히 끝날 때까지 최대 2초 대기합니다.
        spin_thread.join(timeout=2.0)

        # node.destroy_node() → 노드에 연결된 ROS2 리소스(퍼블리셔, 타이머 등) 해제.
        # rclpy.try_shutdown() → DDS 미들웨어 연결 해제 및 rclpy 종료.
        # 반드시 destroy_node() 가 먼저, rclpy.try_shutdown() 이 나중입니다.
        # 순서가 바뀌면 내부 리소스가 이미 해제된 상태에서 노드를 파괴하려 해 오류 발생 가능.
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
nav_goal_ui.py — PyQt5 GUI + Nav2 액션 클라이언트 노드
========================================================

개요
----
이 파일은 PyQt5 그래픽 UI와 ROS2 ActionClient를 결합하여,
Clearpath J100 로봇에게 내비게이션 목표 좌표를 전달하는 노드입니다.

기능
----
1. "Saved Point" 버튼: 미리 하드코딩된 좌표(SAVED_X, SAVED_Y 등)로 로봇을 이동
2. "Custom Goal" 입력창: 사용자가 x / y / yaw 값을 직접 입력하여 이동

ROS2 노드 정보
--------------
- 노드명          : nav_goal_ui
- 사용하는 액션명  : /j100_0001/navigate_to_pose
- 액션 타입        : nav2_msgs/action/NavigateToPose
  * Goal     → 목표 PoseStamped + behavior_tree 이름 (기본값: 빈 문자열)
  * Feedback → 남은 거리, 현재 포즈, 소요 시간 등 (주행 중 지속 수신)
  * Result   → 도착 완료(SUCCEEDED) / 취소(CANCELED) / 실패(ABORTED) 등 최종 상태

실행 방법
---------
# 방법 1: 단독 실행
ros2 run j100_nav2_bringup nav_goal_ui

# 방법 2: launch 파일에서 자동 실행 (j100_nav2_bringup 패키지의 launch 파일 사용)
ros2 launch j100_nav2_bringup bringup.launch.py
"""

import math
import signal
import sys
import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QPushButton, QLabel,
    QDoubleSpinBox, QFormLayout, QVBoxLayout, QHBoxLayout, QGroupBox,
)
from PyQt5.QtCore import QObject, pyqtSignal, QTimer, Qt


# ─────────────────────────────────────────────────────────────────────────────
# Constants
# ─────────────────────────────────────────────────────────────────────────────

# Nav2 BT(Behaviour Tree) Navigator 액션 서버의 이름.
# 네임스페이스(/j100_0001)가 붙어 있어, 여러 로봇을 동시에 운용할 때 충돌을 방지합니다.
ACTION_NAME = '/j100_0001/navigate_to_pose'

# ── 저장된 목표 좌표 ─────────────────────────────────────────────────────────
#
# 이 값들은 실제 로봇을 시뮬레이션 환경에서 원하는 위치로 수동 이동시킨 뒤
# 아래 명령으로 AMCL(적응형 몬테카를로 위치추정)의 추정 위치를 그대로 읽어온 것입니다:
#
#   ros2 topic echo /j100_0001/amcl_pose --once
#
# echo 결과의 pose.pose 필드 값을 그대로 복사했기 때문에,
# 위치(x, y)는 미터(m) 단위 / 자세(qz, qw)는 쿼터니언 형태입니다.
#
# SAVED_X, SAVED_Y : map 프레임 기준 위치 (단위: m)
# SAVED_QZ, SAVED_QW : 2D 회전을 표현하는 쿼터니언의 z·w 성분
#                      (qx=0, qy=0 고정, 2D 평면 회전은 z/w만 사용)
SAVED_X  =  1.8797
SAVED_Y  = -12.0739
SAVED_QZ = -0.7179
SAVED_QW =  0.6961


# ─────────────────────────────────────────────────────────────────────────────
# StatusBridge — marshals status strings from ROS executor thread → Qt thread
# ─────────────────────────────────────────────────────────────────────────────

# ── 스레드 안전 상태 전달 브릿지 ────────────────────────────────────────────
#
# ROS2 콜백(예: _feedback_cb, _result_cb)은 ROS executor 스레드에서 실행됩니다.
# 반면 PyQt5의 UI 위젯(QLabel 등)은 반드시 Qt 메인 스레드에서만 수정해야 합니다.
# 비-메인 스레드에서 QLabel.setText()를 직접 호출하면 세그폴트 등 예측 불가능한
# 충돌이 발생할 수 있습니다.
#
# Qt의 pyqtSignal은 다른 스레드에서 emit()하더라도 Qt 메인 루프가
# 자동으로 큐에 등록하여 메인 스레드에서 안전하게 처리해 줍니다(Qt::QueuedConnection).
# 즉, StatusBridge는 "ROS 콜백 스레드 → Qt 메인 스레드"로 신호를 안전하게 전달하는
# 경량 브릿지 역할을 합니다.
class StatusBridge(QObject):
    # pyqtSignal(str): str 타입 인자를 하나 전달하는 Qt 시그널.
    # emit(문자열)로 발신하면 연결된 슬롯(여기서는 QLabel.setText)이
    # Qt 메인 스레드에서 자동 호출됩니다.
    status_changed = pyqtSignal(str)


# ─────────────────────────────────────────────────────────────────────────────
# ROS2 action-client node
# ─────────────────────────────────────────────────────────────────────────────

# NavGoalNode는 rclpy.node.Node를 상속합니다.
# ROS2에서 모든 노드는 Node 클래스를 상속해야만 토픽, 서비스, 액션 등
# ROS2 미들웨어(DDS) 기능을 사용할 수 있습니다.
class NavGoalNode(Node):
    def __init__(self):
        # super().__init__('nav_goal_ui') — 노드명을 'nav_goal_ui'로 ROS2 네트워크에 등록.
        # 이 이름은 ros2 node list 명령으로 확인할 수 있습니다.
        super().__init__('nav_goal_ui')

        # ── 액션 클라이언트 생성 ──────────────────────────────────────────────
        #
        # ActionClient(self, NavigateToPose, ACTION_NAME)
        #
        # ▶ ROS2 통신 프로토콜 비교
        #
        #   토픽(Topic):
        #     - 단방향 publish/subscribe 스트림.
        #     - 발신자는 수신자의 응답을 기다리지 않습니다(fire-and-forget).
        #     - 예: /cmd_vel (속도 명령), /scan (라이다 데이터)
        #
        #   서비스(Service):
        #     - 동기식 request/response 1회 교환.
        #     - 클라이언트가 요청을 보내면 서버의 응답이 올 때까지 블로킹됩니다.
        #     - 빠르게 끝나는 작업에 적합합니다.
        #     - 예: /set_parameters, /clear_costmaps
        #
        #   액션(Action) ← 이 파일에서 사용하는 방식:
        #     - 비동기 long-running 작업을 위한 프로토콜.
        #     - Goal 전송 → (즉시 future 반환) → 서버가 accept/reject → accept 시
        #       Feedback 스트림(주행 중 지속 수신) → 최종 Result(도착/실패/취소).
        #     - 작업 도중 cancel을 보낼 수도 있습니다.
        #     - 예: 내비게이션, 팔 모션, 장시간 스캔 등
        #
        # ▶ 인자 설명
        #   self           : 이 액션 클라이언트가 소속될 노드 (NavGoalNode 자신)
        #   NavigateToPose : 액션 타입 — nav2_msgs/action/NavigateToPose.
        #                    Goal / Feedback / Result 메시지 구조를 모두 포함합니다.
        #   ACTION_NAME    : 액션 서버의 이름 '/j100_0001/navigate_to_pose'.
        #                    Nav2의 BT(Behaviour Tree) Navigator가 이 이름으로
        #                    액션 서버를 열고 있습니다.
        self._action_client = ActionClient(self, NavigateToPose, ACTION_NAME)

        # ROS 콜백 → Qt 메인 스레드로 상태 문자열을 전달하는 브릿지 (위 설명 참조)
        self.bridge = StatusBridge()

        # _goal_handle: 현재 진행 중인 goal의 핸들.
        # None이면 진행 중인 goal이 없음을 의미합니다.
        # accept 후 get_result_async()를 등록하거나 cancel_goal_async()를 호출할 때 사용합니다.
        self._goal_handle = None

        # _lock: ROS executor 스레드(콜백)와 Qt 스레드(버튼 핸들러)가
        # _goal_handle 및 _terminal에 동시에 접근할 수 있으므로
        # threading.Lock으로 임계 구역을 보호합니다.
        self._lock = threading.Lock()

        # _terminal: goal이 최종 상태(SUCCEEDED / CANCELED / ABORTED)에 도달했는지 여부.
        # True이면 이미 종료된 goal이므로 cancel을 보내도 의미가 없습니다.
        # race condition 방지 목적으로 _lock 안에서만 읽고 씁니다.
        self._terminal = False

    def send_goal(self, x, y, qz, qw):
        """
        Nav2 액션 서버에 내비게이션 목표를 전송합니다.

        매개변수
        --------
        x   : float — map 프레임 기준 목표 x 좌표 (단위: m, 미터)
        y   : float — map 프레임 기준 목표 y 좌표 (단위: m, 미터)
        qz  : float — 목표 방향의 쿼터니언 z 성분 = sin(yaw/2)
        qw  : float — 목표 방향의 쿼터니언 w 성분 = cos(yaw/2)

        주의: x, y는 반드시 미터 단위입니다. 도(degree)와 혼동 금지.
              yaw는 라디안 단위이며, GUI 입력란에 -1.597 같은 값을 직접 입력합니다.
        """
        # wait_for_server: 액션 서버(/j100_0001/navigate_to_pose)가 살아 있는지 확인.
        # timeout_sec=1.0 이내에 응답이 없으면 False를 반환합니다.
        # Nav2가 완전히 기동되기 전에 버튼을 누르면 이 경로로 빠져나옵니다.
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.bridge.status_changed.emit('SERVER NOT READY')
            return

        # 새 goal을 보내기 전에 이전 goal의 상태를 초기화합니다.
        # _lock으로 보호하여 ROS 콜백 스레드와의 race condition을 방지합니다.
        with self._lock:
            self._terminal = False
            self._goal_handle = None

        # ── NavigateToPose 액션 메시지 구조 ─────────────────────────────────
        #
        # nav2_msgs/action/NavigateToPose 액션 정의:
        #
        # # Goal 부분 (클라이언트 → 서버)
        # geometry_msgs/PoseStamped pose
        #   std_msgs/Header header
        #     string frame_id          # 보통 'map' (전역 좌표계)
        #     builtin_interfaces/Time stamp
        #   geometry_msgs/Pose pose
        #     Point position
        #       float64 x              # m, map 프레임 기준
        #       float64 y              # m
        #       float64 z              # 0 (2D 환경)
        #     Quaternion orientation   # 4D 쿼터니언 — yaw → (qz=sin(yaw/2), qw=cos(yaw/2))
        #       float64 x              # 0 (2D 환경)
        #       float64 y              # 0 (2D 환경)
        #       float64 z              # sin(yaw/2)
        #       float64 w              # cos(yaw/2)
        # string behavior_tree         # 빈 문자열이면 default BT 사용
        # ---
        # # Result 부분 (서버 → 클라이언트, 최종)
        # std_msgs/Empty result        # 단순 도착 시그널 (별도 데이터 없음)
        # ---
        # # Feedback 부분 (서버 → 클라이언트, 진행 중 스트림)
        # geometry_msgs/PoseStamped current_pose
        # builtin_interfaces/Duration navigation_time
        # builtin_interfaces/Duration estimated_time_remaining
        # int16 number_of_recoveries   # spin/clear 등 복구 행동 발동 횟수
        # float32 distance_remaining   # m, 목적지까지 남은 거리

        goal = NavigateToPose.Goal()
        pose = PoseStamped()

        # frame_id = 'map': 이 좌표가 map 전역 좌표계 기준임을 명시합니다.
        # 'odom'이나 'base_link' 프레임으로 보내면 Nav2가 거부하거나 오동작합니다.
        pose.header.frame_id = 'map'

        # 현재 ROS 시각을 타임스탬프로 설정합니다.
        # stamp가 오래된 값이면 Nav2가 TF 타임아웃으로 goal을 거부할 수 있습니다.
        pose.header.stamp = self.get_clock().now().to_msg()

        # 목표 위치 설정 (단위: m)
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0          # 2D 주행이므로 항상 0

        # 목표 방향 설정 (쿼터니언)
        # 2D 평면 회전은 z, w 성분만 사용합니다. x=0, y=0으로 고정.
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = float(qz)  # sin(yaw / 2)
        pose.pose.orientation.w = float(qw)  # cos(yaw / 2)
        goal.pose = pose

        # UI에 즉시 'PENDING' 상태 표시 — 서버 응답을 기다리는 중임을 알립니다.
        self.bridge.status_changed.emit('PENDING')

        # ── send_goal_async 호출 흐름 ────────────────────────────────────────
        # 1. send_goal_async(goal, feedback_callback=...) 를 호출하면
        #    즉시 Future 객체를 반환합니다 (블로킹 없음).
        # 2. ROS executor가 내부적으로 서버와 핸드셰이크를 수행합니다.
        # 3. 서버가 goal을 accept 또는 reject하면 _goal_response_cb가 호출됩니다.
        # 4. accept된 경우 _goal_response_cb에서 get_result_async()를 등록합니다.
        # 5. 로봇이 목적지에 도착하거나 실패하면 _result_cb가 호출됩니다.
        # 6. 주행 중에는 feedback_callback(_feedback_cb)이 주기적으로 호출됩니다.
        send_future = self._action_client.send_goal_async(
            goal, feedback_callback=self._feedback_cb
        )
        send_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        """
        서버가 goal을 수락(accept)하거나 거부(reject)했을 때 호출되는 콜백.

        ROS executor 스레드에서 실행됩니다.
        """
        goal_handle = future.result()

        # accepted가 False이면 서버가 goal을 거부한 것입니다.
        # Nav2가 아직 준비되지 않았거나 다른 이유로 거부할 수 있습니다.
        if not goal_handle.accepted:
            self.bridge.status_changed.emit('REJECTED')
            return

        # goal이 수락되면 _goal_handle에 저장합니다.
        # _lock으로 보호: Qt 버튼 스레드에서 cancel_goal()이 동시에
        # _goal_handle을 읽을 수 있으므로 임계 구역을 잠급니다.
        with self._lock:
            self._goal_handle = goal_handle

        self.bridge.status_changed.emit('ACTIVE')

        # get_result_async(): 최종 결과를 비동기로 기다리는 Future를 등록합니다.
        # 로봇이 목적지에 도착하거나 중단(abort/cancel)되면
        # _result_cb가 호출됩니다.
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback_msg):
        """
        주행 중 Nav2 서버가 주기적으로 보내는 Feedback을 처리하는 콜백.

        feedback_msg.feedback에는 다음 필드가 있습니다:
          - current_pose           : 현재 로봇 위치 (PoseStamped)
          - navigation_time        : 내비게이션 시작 후 경과 시간
          - estimated_time_remaining : 목적지까지 예상 남은 시간
          - number_of_recoveries   : spin/clear 등 복구 행동 발동 횟수
          - distance_remaining     : 목적지까지 남은 직선 거리 (단위: m)

        여기서는 distance_remaining만 읽어 UI에 표시합니다.
        """
        # 목적지까지 남은 거리(m)를 소수점 2자리로 반올림하여 상태창에 표시합니다.
        dist = feedback_msg.feedback.distance_remaining
        self.bridge.status_changed.emit(f'ACTIVE — {dist:.2f} m left')

    def _result_cb(self, future):
        """
        goal이 최종 상태(SUCCEEDED / CANCELED / ABORTED)에 도달했을 때 호출되는 콜백.

        ROS executor 스레드에서 실행됩니다.
        """
        # _terminal = True로 설정하여 이 goal이 이미 종료됐음을 기록합니다.
        # 이후 cancel_goal()을 호출해도 "NOTHING TO CANCEL"로 처리됩니다.
        # _lock으로 Qt 스레드와의 동시 접근을 방지합니다.
        with self._lock:
            self._terminal = True

        # future.result().status: GoalStatus 상수값으로 최종 결과를 확인합니다.
        # STATUS_SUCCEEDED : 목적지 도착 완료
        # STATUS_CANCELED  : cancel_goal_async() 요청이 수락되어 중단됨
        # STATUS_ABORTED   : Nav2 내부 오류(경로 없음, 장애물 등)로 실패
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            text = 'SUCCEEDED'
        elif status == GoalStatus.STATUS_CANCELED:
            text = 'CANCELED'
        elif status == GoalStatus.STATUS_ABORTED:
            text = 'ABORTED'
        else:
            # 위 세 가지 이외의 상태는 숫자 코드로 표시합니다.
            text = f'STATUS_{status}'

        self.bridge.status_changed.emit(text)

    def cancel_goal(self):
        """
        현재 진행 중인 goal의 취소를 액션 서버에 요청합니다.

        이미 완료된 goal이거나 진행 중인 goal이 없으면 아무 작업도 하지 않습니다.
        """
        # _lock 안에서 현재 상태를 읽습니다.
        # _goal_handle 과 _terminal을 한 번의 lock으로 원자적으로 읽어
        # 두 값 사이의 race condition을 차단합니다.
        with self._lock:
            handle = self._goal_handle
            terminal = self._terminal

        # handle이 None → 아직 goal을 보내지 않았거나 응답을 받지 못한 상태.
        # terminal이 True → 이미 SUCCEEDED / CANCELED / ABORTED로 종료된 상태.
        # 두 경우 모두 취소할 대상이 없으므로 조기 반환합니다.
        if handle is None or terminal:
            self.bridge.status_changed.emit('NOTHING TO CANCEL')
            return

        # cancel_goal_async(): 비동기로 취소 요청을 서버에 보냅니다.
        # 서버가 취소를 수락하면 _result_cb가 STATUS_CANCELED로 호출됩니다.
        handle.cancel_goal_async()
        self.bridge.status_changed.emit('CANCELING...')


# ─────────────────────────────────────────────────────────────────────────────
# Qt main window
# ─────────────────────────────────────────────────────────────────────────────
class MainWindow(QMainWindow):
    def __init__(self, node: NavGoalNode):
        super().__init__()
        self.node = node

        self.setWindowTitle('J100 Nav2 Goal Sender')
        self.resize(380, 420)

        # ── Title label ──────────────────────────────────────────────────────
        lbl_title = QLabel('<b>J100 Navigation Goal Sender</b>')
        lbl_title.setAlignment(Qt.AlignCenter)

        # ── Saved Point group ────────────────────────────────────────────────
        grp_saved = QGroupBox('Saved Point')
        lbl_coords = QLabel(
            f'x={SAVED_X}, y={SAVED_Y}, qz={SAVED_QZ}, qw={SAVED_QW}'
        )
        lbl_coords.setAlignment(Qt.AlignCenter)
        btn_send_saved = QPushButton('Send to Saved Point')
        btn_send_saved.clicked.connect(self._on_send_saved)

        saved_layout = QVBoxLayout()
        saved_layout.addWidget(lbl_coords)
        saved_layout.addWidget(btn_send_saved)
        grp_saved.setLayout(saved_layout)

        # ── Custom Goal group ────────────────────────────────────────────────
        grp_custom = QGroupBox('Custom Goal')

        self.spin_x = QDoubleSpinBox()
        self.spin_x.setRange(-50.0, 50.0)
        self.spin_x.setSingleStep(0.1)
        self.spin_x.setDecimals(3)
        self.spin_x.setValue(0.0)

        self.spin_y = QDoubleSpinBox()
        self.spin_y.setRange(-50.0, 50.0)
        self.spin_y.setSingleStep(0.1)
        self.spin_y.setDecimals(3)
        self.spin_y.setValue(0.0)

        self.spin_yaw = QDoubleSpinBox()
        self.spin_yaw.setRange(-3.14159, 3.14159)
        self.spin_yaw.setSingleStep(0.1)
        self.spin_yaw.setDecimals(4)
        self.spin_yaw.setValue(0.0)

        btn_send_custom = QPushButton('Send Custom Goal')
        btn_send_custom.clicked.connect(self._on_send_custom)

        custom_form = QFormLayout()
        custom_form.addRow('X (m):', self.spin_x)
        custom_form.addRow('Y (m):', self.spin_y)
        custom_form.addRow('Yaw (rad):', self.spin_yaw)
        custom_form.addRow(btn_send_custom)
        grp_custom.setLayout(custom_form)

        # ── Status group ─────────────────────────────────────────────────────
        grp_status = QGroupBox('Status')

        self.lbl_status = QLabel('IDLE')
        status_font = self.lbl_status.font()
        status_font.setPointSize(14)
        status_font.setBold(True)
        self.lbl_status.setFont(status_font)
        self.lbl_status.setAlignment(Qt.AlignCenter)

        btn_cancel = QPushButton('Cancel Goal')
        btn_cancel.clicked.connect(self._on_cancel)

        status_layout = QVBoxLayout()
        status_layout.addWidget(self.lbl_status)
        status_layout.addWidget(btn_cancel)
        grp_status.setLayout(status_layout)

        # ── Root layout ──────────────────────────────────────────────────────
        root = QVBoxLayout()
        root.addWidget(lbl_title)
        root.addWidget(grp_saved)
        root.addWidget(grp_custom)
        root.addWidget(grp_status)

        container = QWidget()
        container.setLayout(root)
        container.setContentsMargins(12, 12, 12, 12)
        self.setCentralWidget(container)

        # ROS StatusBridge의 status_changed 시그널을 Qt 라벨의 setText 슬롯에 연결합니다.
        # Qt는 시그널이 다른 스레드에서 emit됐음을 감지하면 자동으로
        # Qt::QueuedConnection 방식으로 큐에 등록하여 메인 스레드에서 실행합니다.
        # 따라서 ROS 콜백 스레드가 emit()을 호출해도 setText()는 항상
        # Qt 메인 스레드에서 안전하게 실행됩니다.
        node.bridge.status_changed.connect(self.lbl_status.setText)

    # ── Button handlers ───────────────────────────────────────────────────────

    def _on_send_saved(self):
        """'Send to Saved Point' 버튼 클릭 시 호출됩니다.
        파일 상단의 SAVED_* 상수(AMCL echo 결과)를 그대로 goal로 전송합니다."""
        self.node.send_goal(SAVED_X, SAVED_Y, SAVED_QZ, SAVED_QW)

    def _on_send_custom(self):
        """
        'Send Custom Goal' 버튼 클릭 시 호출됩니다.
        사용자가 입력한 x(m), y(m), yaw(rad)를 읽어 쿼터니언으로 변환 후 전송합니다.
        """
        # spin 위젯에서 현재 값을 읽습니다.
        # x, y 단위: m (미터), map 프레임 기준. 절대 도(degree)가 아님에 주의.
        x = self.spin_x.value()
        y = self.spin_y.value()

        # yaw 단위: rad (라디안). 범위: -π ~ +π.
        # 예) 0 = 동쪽, π/2 ≈ 1.5708 = 북쪽, -π/2 ≈ -1.5708 = 남쪽
        yaw = self.spin_yaw.value()

        # ── yaw → 쿼터니언 변환 ──────────────────────────────────────────────
        # 왜 쿼터니언인가?
        #   오일러 각(roll/pitch/yaw)은 짐벌 락(gimbal lock) 문제와 보간 불안정성이 있어
        #   로보틱스에서는 쿼터니언(4D 단위 벡터)으로 자세를 표현하는 것이 표준입니다.
        #
        # 2D 평면(ground plane) 회전이므로 roll=0, pitch=0, yaw만 사용합니다.
        # 이 경우 쿼터니언 변환식은 다음과 같이 단순화됩니다:
        #   qx = 0
        #   qy = 0
        #   qz = sin(yaw / 2)   ← 회전축 z, 회전각 yaw의 절반에 대한 사인
        #   qw = cos(yaw / 2)   ← 회전각 yaw의 절반에 대한 코사인 (스칼라 성분)
        qz = math.sin(yaw / 2.0)   # sin(yaw/2): 쿼터니언 z 성분
        qw = math.cos(yaw / 2.0)   # cos(yaw/2): 쿼터니언 w 성분(스칼라)
        self.node.send_goal(x, y, qz, qw)

    def _on_cancel(self):
        """'Cancel Goal' 버튼 클릭 시 현재 진행 중인 goal의 취소를 요청합니다."""
        self.node.cancel_goal()


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────
def main():
    """
    프로그램 진입점.

    ROS2 초기화 → 노드 생성 → executor 스레드 시작 → Qt 이벤트 루프 실행
    → Qt 종료 후 ROS2 정리 순서로 동작합니다.
    """
    # rclpy.init(): ROS2 미들웨어(DDS)를 초기화합니다.
    # 이 호출 전까지는 노드를 생성하거나 토픽/서비스/액션을 사용할 수 없습니다.
    rclpy.init()

    # NavGoalNode를 생성합니다 (ActionClient 포함).
    node = NavGoalNode()

    # ── ROS executor와 Qt 메인 루프를 분리하는 패턴 ─────────────────────────
    #
    # 문제: Qt의 app.exec_()는 메인 스레드를 점유하는 이벤트 루프입니다.
    #       동시에 rclpy의 executor.spin()도 무한 루프입니다.
    #       두 루프를 같은 스레드에서 돌릴 수 없습니다.
    #
    # 해결책: ROS executor를 별도의 데몬 스레드에서 실행합니다.
    #   - 메인 스레드: Qt 이벤트 루프(app.exec_())
    #   - 백그라운드 스레드: ROS executor spin (콜백 처리)
    #
    # SingleThreadedExecutor: 노드의 콜백을 단일 스레드에서 순차 실행합니다.
    # 멀티스레드 콜백이 필요하면 MultiThreadedExecutor로 교체할 수 있습니다.
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    # daemon=True: 메인 스레드(Qt)가 종료되면 이 스레드도 자동으로 종료됩니다.
    # daemon=False였다면 Qt 창을 닫아도 ROS spin이 계속 돌아 프로세스가 안 끝납니다.
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Qt 애플리케이션 객체를 생성합니다. sys.argv를 넘겨 Qt 자체 옵션을 처리합니다.
    app = QApplication(sys.argv)

    # Ctrl+C(SIGINT) 처리: 터미널에서 Ctrl+C를 누르면 app.quit()를 호출해
    # Qt 이벤트 루프를 정상 종료합니다.
    signal.signal(signal.SIGINT, lambda *_: app.quit())

    # sigint_kicker: Python의 signal 핸들러가 발화되려면 Python 인터프리터가
    # 주기적으로 제어권을 되찾아야 합니다. Qt 이벤트 루프는 기본적으로 Python에게
    # 제어권을 넘기지 않으므로, 100ms마다 빈 타이머를 돌려 SIGINT를 처리할 기회를 줍니다.
    sigint_kicker = QTimer(app)
    sigint_kicker.start(100)
    sigint_kicker.timeout.connect(lambda: None)

    # 메인 윈도우를 생성하고 표시합니다.
    window = MainWindow(node)
    window.show()

    try:
        # Qt 메인 이벤트 루프를 시작합니다. 창을 닫거나 app.quit()가 호출될 때까지 블로킹됩니다.
        app.exec_()
    finally:
        # ── 종료 정리 시퀀스 ────────────────────────────────────────────────
        # Qt 창이 닫힌 후 ROS 자원을 순서대로 정리합니다.

        # 1. 진행 중인 goal이 있다면 취소 시도 (선택적, 실패해도 무시)
        try:
            node.cancel_goal()
        except Exception:
            pass

        # 2. executor를 종료하여 spin 루프를 멈춥니다.
        executor.shutdown()

        # 3. spin 스레드가 완전히 종료될 때까지 최대 2초 기다립니다.
        spin_thread.join(timeout=2.0)

        # 4. 노드를 소멸시켜 DDS 참여자를 해제합니다.
        node.destroy_node()

        # 5. rclpy 미들웨어를 종료합니다. 이미 종료됐어도 안전하게 처리합니다(try_shutdown).
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

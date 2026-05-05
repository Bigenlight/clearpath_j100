#!/usr/bin/env python3
"""PyQt5 GUI node for sending Nav2 goals to a Clearpath J100 robot in simulation."""

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
ACTION_NAME = '/j100_0001/navigate_to_pose'

SAVED_X  =  1.8797
SAVED_Y  = -12.0739
SAVED_QZ = -0.7179
SAVED_QW =  0.6961


# ─────────────────────────────────────────────────────────────────────────────
# StatusBridge — marshals status strings from ROS executor thread → Qt thread
# ─────────────────────────────────────────────────────────────────────────────
class StatusBridge(QObject):
    status_changed = pyqtSignal(str)


# ─────────────────────────────────────────────────────────────────────────────
# ROS2 action-client node
# ─────────────────────────────────────────────────────────────────────────────
class NavGoalNode(Node):
    def __init__(self):
        super().__init__('nav_goal_ui')
        self._action_client = ActionClient(self, NavigateToPose, ACTION_NAME)
        self.bridge = StatusBridge()
        self._goal_handle = None
        self._lock = threading.Lock()
        self._terminal = False

    def send_goal(self, x, y, qz, qw):
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.bridge.status_changed.emit('SERVER NOT READY')
            return

        with self._lock:
            self._terminal = False
            self._goal_handle = None

        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = float(qz)
        pose.pose.orientation.w = float(qw)
        goal.pose = pose

        self.bridge.status_changed.emit('PENDING')

        send_future = self._action_client.send_goal_async(
            goal, feedback_callback=self._feedback_cb
        )
        send_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.bridge.status_changed.emit('REJECTED')
            return

        with self._lock:
            self._goal_handle = goal_handle

        self.bridge.status_changed.emit('ACTIVE')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        self.bridge.status_changed.emit(f'ACTIVE — {dist:.2f} m left')

    def _result_cb(self, future):
        with self._lock:
            self._terminal = True

        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            text = 'SUCCEEDED'
        elif status == GoalStatus.STATUS_CANCELED:
            text = 'CANCELED'
        elif status == GoalStatus.STATUS_ABORTED:
            text = 'ABORTED'
        else:
            text = f'STATUS_{status}'

        self.bridge.status_changed.emit(text)

    def cancel_goal(self):
        with self._lock:
            handle = self._goal_handle
            terminal = self._terminal

        if handle is None or terminal:
            self.bridge.status_changed.emit('NOTHING TO CANCEL')
            return

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

        # Connect ROS status bridge to Qt label (auto-queued across threads)
        node.bridge.status_changed.connect(self.lbl_status.setText)

    # ── Button handlers ───────────────────────────────────────────────────────
    def _on_send_saved(self):
        self.node.send_goal(SAVED_X, SAVED_Y, SAVED_QZ, SAVED_QW)

    def _on_send_custom(self):
        x = self.spin_x.value()
        y = self.spin_y.value()
        yaw = self.spin_yaw.value()
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        self.node.send_goal(x, y, qz, qw)

    def _on_cancel(self):
        self.node.cancel_goal()


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = NavGoalNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    app = QApplication(sys.argv)
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    sigint_kicker = QTimer(app)
    sigint_kicker.start(100)
    sigint_kicker.timeout.connect(lambda: None)

    window = MainWindow(node)
    window.show()
    try:
        app.exec_()
    finally:
        try:
            node.cancel_goal()
        except Exception:
            pass
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

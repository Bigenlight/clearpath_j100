#!/usr/bin/env python3
"""PyQt5 virtual joystick → /j100_0001/cmd_vel publisher.

Run:
    source /opt/ros/humble/setup.bash
    python3 joystick_ui.py
"""
import math
import signal
import sys
import threading

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from PyQt5.QtCore import Qt, QEvent, QPointF, QTimer, pyqtSignal
from PyQt5.QtGui import QBrush, QColor, QKeySequence, QPainter, QPen, QRadialGradient
from PyQt5.QtWidgets import (
    QApplication, QHBoxLayout, QLabel, QMainWindow, QPushButton,
    QShortcut, QSlider, QVBoxLayout, QWidget,
)


CMD_VEL_TOPIC = '/j100_0001/cmd_vel'
PUBLISH_PERIOD_MS = 50          # 20 Hz, well under twist_mux 0.5 s timeout
DEFAULT_MAX_LINEAR = 0.5        # m/s
DEFAULT_MAX_ANGULAR = 1.0       # rad/s
HARD_MAX_LINEAR = 4.0           # slider upper bound (controller clamps at 2.0 by robot.yaml)
HARD_MAX_ANGULAR = 8.0          # slider upper bound (controller clamps at 4.0 by robot.yaml)
AXIS_SNAP_FRAC = 0.15           # snap to pure axis when within 15% of cardinal


# ────────────────────────────────────────────────────────────────────────
# Joystick widget
# ────────────────────────────────────────────────────────────────────────
class JoystickWidget(QWidget):
    """Spring-centered 2D joystick. Emits normalized (x_right, y_up) ∈ [-1, 1]."""

    position_changed = pyqtSignal(float, float)

    def __init__(self, parent=None, size=300):
        super().__init__(parent)
        self.setFixedSize(size, size)
        self._outer_radius = size * 0.45
        self._knob_radius = self._outer_radius * 0.25
        self._max_offset = self._outer_radius - self._knob_radius
        self._knob_pos = QPointF(0.0, 0.0)   # offset from center, Qt coords (y-down)
        self._dragging = False

    def _center(self) -> QPointF:
        return QPointF(self.width() / 2.0, self.height() / 2.0)

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        c = self._center()

        # outer dish
        grad = QRadialGradient(c, self._outer_radius)
        grad.setColorAt(0.0, QColor(60, 65, 75))
        grad.setColorAt(1.0, QColor(30, 32, 38))
        p.setBrush(QBrush(grad))
        p.setPen(QPen(QColor(20, 22, 26), 2))
        p.drawEllipse(c, self._outer_radius, self._outer_radius)

        # crosshair
        p.setPen(QPen(QColor(110, 115, 125), 1, Qt.DashLine))
        p.drawLine(QPointF(c.x() - self._max_offset, c.y()),
                   QPointF(c.x() + self._max_offset, c.y()))
        p.drawLine(QPointF(c.x(), c.y() - self._max_offset),
                   QPointF(c.x(), c.y() + self._max_offset))
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(140, 145, 155))
        p.drawEllipse(c, 3, 3)

        # knob
        kpos = c + self._knob_pos
        knob_color = QColor(90, 170, 240) if self._dragging else QColor(180, 190, 205)
        edge_color = QColor(40, 90, 140) if self._dragging else QColor(80, 90, 105)
        kgrad = QRadialGradient(kpos, self._knob_radius)
        kgrad.setColorAt(0.0, knob_color.lighter(120))
        kgrad.setColorAt(1.0, knob_color)
        p.setBrush(QBrush(kgrad))
        p.setPen(QPen(edge_color, 2))
        p.drawEllipse(kpos, self._knob_radius, self._knob_radius)

    def _set_knob_from_mouse(self, pos):
        c = self._center()
        dx, dy = pos.x() - c.x(), pos.y() - c.y()
        dist = math.hypot(dx, dy)
        if dist > self._max_offset:
            dx = dx * self._max_offset / dist
            dy = dy * self._max_offset / dist

        # axis snap: pure straight / pure turn made easy
        snap_thresh = self._max_offset * AXIS_SNAP_FRAC
        if abs(dx) < snap_thresh:
            dx = 0.0
        if abs(dy) < snap_thresh:
            dy = 0.0

        self._knob_pos = QPointF(dx, dy)
        self.update()
        nx = dx / self._max_offset
        ny = dy / self._max_offset
        self.position_changed.emit(nx, -ny)   # flip y so up = +1

    def mousePressEvent(self, e):
        if e.button() == Qt.LeftButton:
            self._dragging = True
            self._set_knob_from_mouse(e.pos())

    def mouseMoveEvent(self, e):
        if self._dragging:
            self._set_knob_from_mouse(e.pos())

    def mouseReleaseEvent(self, e):
        if e.button() == Qt.LeftButton and self._dragging:
            self._dragging = False
            self._knob_pos = QPointF(0.0, 0.0)
            self.update()
            self.position_changed.emit(0.0, 0.0)

    def force_zero(self):
        """Snap to zero and emit. Used on app-deactivate / e-stop."""
        if self._dragging or self._knob_pos != QPointF(0.0, 0.0):
            self._dragging = False
            self._knob_pos = QPointF(0.0, 0.0)
            self.update()
            self.position_changed.emit(0.0, 0.0)


# ────────────────────────────────────────────────────────────────────────
# ROS2 node
# ────────────────────────────────────────────────────────────────────────
class TeleopNode(Node):
    def __init__(self):
        super().__init__('joystick_ui')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.publisher = self.create_publisher(Twist, CMD_VEL_TOPIC, qos)

    def publish_twist(self, linear_x: float, angular_z: float) -> bool:
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        try:
            self.publisher.publish(msg)
            return True
        except Exception:
            return False

    def subscriber_count(self) -> int:
        try:
            return self.publisher.get_subscription_count()
        except Exception:
            return -1


# ────────────────────────────────────────────────────────────────────────
# Main window
# ────────────────────────────────────────────────────────────────────────
class MainWindow(QMainWindow):
    def __init__(self, node: TeleopNode):
        super().__init__()
        self.node = node
        self.setWindowTitle('J100 Joystick Control')

        # state
        self._joy_x = 0.0
        self._joy_y = 0.0
        self._max_linear = DEFAULT_MAX_LINEAR
        self._max_angular = DEFAULT_MAX_ANGULAR
        self._estopped = False
        self._publish_count = 0
        self._last_publish_ok = True

        # ── E-STOP (top, prominent) ─────────────────────────────────────
        self.btn_estop = QPushButton('EMERGENCY STOP   (Space)')
        self.btn_estop.setMinimumHeight(60)
        self.btn_estop.setCheckable(True)
        self.btn_estop.toggled.connect(self._on_estop)
        self._apply_estop_style(False)

        # Spacebar shortcut
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

        # publish loop on Qt thread
        self.publish_timer = QTimer(self)
        self.publish_timer.timeout.connect(self._tick_publish)
        self.publish_timer.start(PUBLISH_PERIOD_MS)

        # startup safety zero
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

    # ── slots ─────────────────────────────────────────────────────────
    def _on_joystick(self, x: float, y: float):
        self._joy_x = x
        self._joy_y = y

    def _on_max_linear(self, v: int):
        self._max_linear = v / 100.0
        self.lbl_max_linear.setText(f'Max linear:  {self._max_linear:.2f} m/s')

    def _on_max_angular(self, v: int):
        self._max_angular = v / 100.0
        self.lbl_max_angular.setText(f'Max angular: {self._max_angular:.2f} rad/s')

    def _on_estop(self, on: bool):
        self._estopped = on
        self._apply_estop_style(on)
        if on:
            self.joystick.force_zero()
            self.btn_estop.setText('E-STOP ENGAGED — click or press Space to release')
        else:
            self.btn_estop.setText('EMERGENCY STOP   (Space)')

    # ── publish loop ──────────────────────────────────────────────────
    def _tick_publish(self):
        if self._estopped:
            lin, ang = 0.0, 0.0
        else:
            lin = self._joy_y * self._max_linear     # joystick up = forward
            ang = -self._joy_x * self._max_angular   # joystick right = clockwise = -yaw (REP-103)

        self._last_publish_ok = self.node.publish_twist(lin, ang)
        self._publish_count += 1

        self.lbl_linear.setText(f'{lin:+.3f}  m/s')
        self.lbl_angular.setText(f'{ang:+.3f}  rad/s')

        # health: subscriber count
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

    # ── safety hooks ──────────────────────────────────────────────────
    def changeEvent(self, e):
        # Zero only on app-level deactivate (alt-tab, lose focus to other app).
        # NOT on focus changes between widgets in the same window (slider clicks).
        if e.type() == QEvent.ApplicationStateChange:
            if QApplication.applicationState() != Qt.ApplicationActive:
                self.joystick.force_zero()
        super().changeEvent(e)

    def closeEvent(self, e):
        self.publish_timer.stop()
        self.node.publish_twist(0.0, 0.0)
        QApplication.instance().quit()
        super().closeEvent(e)


# ────────────────────────────────────────────────────────────────────────
# Entry
# ────────────────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = TeleopNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    app = QApplication(sys.argv)
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    sigint_kicker = QTimer(app)            # parented so it isn't GC'd
    sigint_kicker.start(100)
    sigint_kicker.timeout.connect(lambda: None)

    window = MainWindow(node)
    window.show()
    try:
        app.exec_()
    finally:
        # safety zero BEFORE tearing down ROS
        try:
            node.publish_twist(0.0, 0.0)
        except Exception:
            pass
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

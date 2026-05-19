#!/usr/bin/env python3
"""
teleop_split.launch.py — 2-노드 텔레옵 통합 실행 launch 파일
============================================================

[이 파일이 하는 일]
  분리된 두 노드(Node 1 joystick_publisher, Node 2 cmd_vel_relay)를
  한 번의 명령으로 동시에 띄웁니다. 터미널을 두 개 열어
  `ros2 run` 을 각각 칠 필요 없이 다음 한 줄이면 됩니다.

    $ source /opt/ros/humble/setup.bash      # ROS2 환경 (한 번만)
    $ source install/setup.bash              # 빌드 후 오버레이
    $ ros2 launch j100_teleop teleop_split.launch.py

[왜 launch 파일을 따로 두는가]
  모놀리식(joystick_ui.py)은 노드가 1개라 `ros2 run` 한 줄로 끝났지만,
  2-노드 구조에서는 joystick_publisher 와 cmd_vel_relay 가 반드시
  '함께' 떠 있어야 로봇이 움직입니다(둘 중 하나만 띄우면 의미 없음).
  launch 파일은 이 "여러 노드를 한 번에, 일관된 설정으로" 띄우는
  ROS2 표준 메커니즘입니다.

[2-노드 데이터 흐름 — 이 launch 가 띄우는 두 노드의 관계]

    ┌────────────────────┐  /joystick_cmd   ┌──────────────────┐  /j100_0001/cmd_vel  ┌──────────┐
    │ joystick_publisher │ ───────────────▶ │  cmd_vel_relay   │ ───────────────────▶ │   J100   │
    │   (Node 1, Pub)    │ geometry_msgs/   │   (Node 2)       │  geometry_msgs/      │ 로봇/시뮬 │
    │  · PyQt5 GUI       │ msg/Twist        │  · 워치독        │  msg/Twist           └──────────┘
    │  · 단일 스레드     │                  │  · 멀티 스레드   │
    └────────────────────┘                  └──────────────────┘

  · Node 1 (joystick_publisher)
      PyQt5 가상 조이스틱 GUI 를 띄우고, 사용자가 만든 입력을
      스케일링한 Twist 를 중간 토픽 /joystick_cmd 로 20Hz 발행.
  · /joystick_cmd
      두 노드를 잇는 중간 토픽 (geometry_msgs/msg/Twist).
  · Node 2 (cmd_vel_relay)
      /joystick_cmd 를 구독해 워치독을 적용한 뒤
      로봇이 실제로 듣는 /j100_0001/cmd_vel 로 20Hz 재발행.
      입력이 0.5 s 이상 끊기면 zero Twist 를 능동 발행해 로봇 정지.

[ROS2 launch 파일 구조]
  ROS2 의 Python launch 파일은 generate_launch_description() 라는
  함수를 반드시 정의하고, 그 함수가 launch.LaunchDescription 객체를
  반환해야 합니다. `ros2 launch` 가 이 함수를 호출하여 안에 담긴
  Action 들(여기서는 Node 2개)을 실행합니다.
"""
# ── ROS2 launch 핵심 모듈 ──────────────────────────────────────────────────
# launch.LaunchDescription : 실행할 Action 들의 목록을 담는 최상위 컨테이너.
import launch
# launch_ros.actions.Node : ROS2 노드 하나를 실행하는 launch Action.
#   'ros2 run <pkg> <executable>' 에 해당하는 것을 launch 안에서 표현한다.
import launch_ros.actions


def generate_launch_description():
    """`ros2 launch` 가 호출하는 진입 함수.

    LaunchDescription 안에 Node Action 2개(Node 1, Node 2)를 담아
    반환하면, ros2 launch 가 두 노드를 동시에 실행합니다.
    """

    # ── Node 1: joystick_publisher (Pub 전용 + PyQt5 GUI) ────────────────
    # package    : 이 실행 파일이 속한 ROS2 패키지 이름.
    # executable : setup.py 의 console_scripts 에 등록한 실행명.
    #              ('joystick_publisher = j100_teleop.joystick_publisher:main')
    # name       : 실행 시 ROS2 그래프에 등록될 노드 이름.
    #              노드 코드 안의 super().__init__('joystick_publisher') 와
    #              일치시켜 `ros2 node list` 출력을 예측 가능하게 한다.
    # output='screen' : 노드의 stdout/stderr(로그)를 이 터미널에 그대로
    #              출력한다. (기본값 'log' 면 로그 파일로만 가서 화면에 안 보임)
    # 이 노드가 /joystick_cmd 토픽으로 스케일링된 Twist 를 20Hz 발행한다.
    joystick_publisher_node = launch_ros.actions.Node(
        package='j100_teleop',
        executable='joystick_publisher',
        name='joystick_publisher',
        output='screen',
    )

    # ── Node 2: cmd_vel_relay (구독 → 재발행 + 워치독, 멀티스레드) ────────
    # /joystick_cmd 를 구독해 /j100_0001/cmd_vel 로 20Hz 재발행하고,
    # 입력이 0.5 s 이상 끊기면 zero Twist 를 능동 발행해 로봇을 정지시킨다.
    # 내부적으로 MultiThreadedExecutor + 콜백 그룹 2개로
    # 구독 콜백과 타이머 콜백이 다른 스레드에서 병렬 실행된다.
    # output='screen' : 워치독/정상 중계 상태 로그를 이 터미널에서 직접 본다.
    cmd_vel_relay_node = launch_ros.actions.Node(
        package='j100_teleop',
        executable='cmd_vel_relay',
        name='cmd_vel_relay',
        output='screen',
    )

    # ── LaunchDescription 반환 ──────────────────────────────────────────
    # 리스트에 담긴 두 Node Action 이 ros2 launch 에 의해 동시에 실행된다.
    # 둘 다 떠 있어야 joystick_publisher → /joystick_cmd → cmd_vel_relay
    # → /j100_0001/cmd_vel 의 데이터 흐름이 완성되어 로봇이 움직인다.
    return launch.LaunchDescription([
        joystick_publisher_node,
        cmd_vel_relay_node,
    ])

"""
j100_teleop/launch/joystick.launch.py
======================================

J100 (Jackal) 로봇용 PyQt5 가상 조이스틱 UI 노드를 실행하는 launch 파일.

[사용 방법]
  ros2 launch j100_teleop joystick.launch.py

[ros2 run 으로도 직접 실행 가능]
  ros2 run j100_teleop joystick_ui

[이 launch 파일이 하는 일]
  joystick_ui 노드를 하나 띄운다.
  노드는 PyQt5 가상 조이스틱 창을 열고,
  사용자 입력을 geometry_msgs/Twist 메시지로 변환하여
  /j100_0001/cmd_vel 토픽으로 발행한다.
"""

# Copyright 2024 Theo <tpingouin@gmail.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# LaunchDescription: ros2 launch가 처리할 액션(Action)들을 담는 컨테이너.
#   generate_launch_description()의 반환값이 반드시 이 타입이어야 한다.
from launch import LaunchDescription

# Node: ROS2 노드를 직접 프로세스로 실행하는 launch 액션.
#   package + executable 조합으로 실행할 노드를 지정하고,
#   name, output, parameters, remappings 등의 옵션을 설정할 수 있다.
from launch_ros.actions import Node


def generate_launch_description():
    """
    ROS2 launch 시스템 규칙:
      - 파일 내에 반드시 이 이름(generate_launch_description)의 함수가 있어야 한다.
      - ros2 launch 명령이 이 함수를 자동으로 호출하고,
        반환된 LaunchDescription을 처리해 모든 액션을 스케줄링한다.
      - 인자(args)를 받지 않는 것이 규약이다.
    """

    # ── joystick_ui 노드 정의 ───────────────────────────────────────────────────
    joystick_ui_node = Node(
        # package: 노드가 속한 ROS2 패키지 이름.
        #   ament_index를 통해 install/lib/j100_teleop/ 경로를 찾는 데 사용된다.
        package='j100_teleop',

        # executable: 실행할 스크립트 이름.
        #   setup.py의 entry_points 좌변('joystick_ui')과 정확히 일치해야 한다.
        #   colcon이 install/lib/j100_teleop/joystick_ui 스크립트를 생성하고,
        #   launch 시스템이 이 경로의 파일을 직접 실행한다.
        executable='joystick_ui',

        # name: ROS2 그래프에 표시될 노드 이름.
        #   joystick_ui.py 내부 TeleopNode가 'joystick_ui'로 자기 이름을 등록하므로,
        #   여기서도 동일하게 'joystick_ui'로 두어 'ros2 run'과 'ros2 launch' 두 경로
        #   모두에서 'ros2 node list' 결과가 '/joystick_ui'로 일관되게 보이도록 함.
        #   지정하지 않으면 노드 코드 내부 이름이 그대로 쓰이므로 사실상 생략해도 OK.
        name='joystick_ui',

        # output: 노드의 stdout/stderr 출력 방향.
        #   'screen' → 현재 터미널(콘솔)에 직접 출력된다.
        #   기본값('log')이면 ~/.ros/log/ 아래 로그 파일에만 기록되고
        #   터미널에는 출력되지 않아 디버깅이 어려우므로 'screen'을 권장한다.
        output='screen',
    )

    # ── LaunchDescription 구성 및 반환 ─────────────────────────────────────────
    return LaunchDescription([
        joystick_ui_node,
    ])

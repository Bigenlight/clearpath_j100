"""
j100_nav2_bringup/launch/bringup.launch.py
=========================================

J100 (Jackal) 시뮬레이션 + Nav2 풀 스택을 한 번에 띄우는 통합 launch 파일.

[패키지 구조]
src/j100_nav2_bringup/
├── package.xml                 # ament_python 메타데이터, 의존성 선언
├── setup.py                    # 빌드 정의, entry_points로 ros2 run 진입점 등록
├── setup.cfg                   # script_dir 설정 (ament_python 표준)
├── resource/
│   └── j100_nav2_bringup       # ament 리소스 마커 (빈 파일이지만 필수)
├── j100_nav2_bringup/
│   ├── __init__.py
│   └── nav_goal_ui.py          # ros2 run 으로 실행되는 Nav 골 UI 노드
└── launch/
    └── bringup.launch.py       # ← 이 파일

[빌드 방법]
cd ~/clearpath_ws
colcon build --symlink-install --packages-select j100_nav2_bringup
source install/setup.bash

[실행 방법]
ros2 launch j100_nav2_bringup bringup.launch.py

[순차 실행 타이밍 (TimerAction)]
  t=0s   : Gazebo 시뮬레이터 (clearpath_gz/simulation.launch.py)
         + RViz (clearpath_viz/view_navigation.launch.py)
  t=10s  : AMCL 로컬라이제이션 (clearpath_nav2_demos/localization.launch.py)
         → 저장된 맵 로드 + 파티클 필터 시작
  t=15s  : Nav2 스택 (clearpath_nav2_demos/nav2.launch.py)
         → BT navigator + planner_server + controller_server + behavior_server
  t=25s  : 자동 initial pose 발행 (ExecuteProcess + ros2 topic pub --once)
         → AMCL이 시작 위치를 알 수 있도록 /j100_0001/initialpose 한 번 쏨
  t=30s  : Nav 골 UI (j100_nav2_bringup의 nav_goal_ui)
         → 모든 게 준비된 후 사용자 GUI 띄움

[왜 순차인가?]
Nav2는 AMCL이 먼저 떠야 하고, AMCL은 시뮬이 떠 있어야 라이다/오도메트리를 받음.
TimerAction 없이 동시에 띄우면 race condition으로 일부 컴포넌트가 죽을 수 있음.
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

# ─── ament_index: 패키지 경로 조회 ───────────────────────────────────────────
# get_package_share_directory('패키지명') 은 런타임에 해당 패키지의 share 디렉터리
# 절대 경로를 반환한다.
# 예: '/opt/ros/humble/share/clearpath_gz'
# 하드코딩 대신 이 함수를 써야 하는 이유:
#   - ROS 배포판(humble/iron 등)마다 경로가 다름
#   - install space가 overlay될 수 있음 (workspace 우선, 시스템 설치 fallback)
#   - colcon --symlink-install 이면 개발 트리를 직접 가리키므로 재빌드 불필요
from ament_index_python.packages import get_package_share_directory

# ─── launch 코어 ─────────────────────────────────────────────────────────────
# LaunchDescription: ros2 launch가 처리할 액션(Action)들을 담는 컨테이너.
#   generate_launch_description()의 반환값이 되어야 한다.
from launch import LaunchDescription

# ─── launch 액션들 ────────────────────────────────────────────────────────────
# DeclareLaunchArgument : 커맨드라인(또는 상위 launch)에서 인자를 선언·기본값 설정
#   예: ros2 launch ... world:=my_world
# ExecuteProcess        : 셸 커맨드를 자식 프로세스로 실행
#   여기서는 ros2 topic pub 으로 initial pose를 한 번 발행할 때 사용
# IncludeLaunchDescription : 다른 .launch.py 파일 전체를 이 launch에 포함(embed)
#   서브시스템별 launch 파일을 모듈로 재사용할 때의 표준 방법
# TimerAction           : 지정한 period(초) 후에 actions 리스트를 실행
#   race condition 방지를 위해 컴포넌트 시작 시점을 순차적으로 분리할 때 사용
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)

# PythonLaunchDescriptionSource: .py 형식의 launch 파일을 IncludeLaunchDescription에
#   넘길 때 사용하는 wrapper. XML launch라면 XMLLaunchDescriptionSource를 쓴다.
from launch.launch_description_sources import PythonLaunchDescriptionSource

# ─── launch 치환자(Substitution) ─────────────────────────────────────────────
# Substitution은 launch 시스템이 "지연 평가(lazy evaluation)"하는 표현식이다.
# Python 변수처럼 바로 값이 확정되지 않고, 실제 액션이 실행될 때 값이 결정된다.
#
# EnvironmentVariable : 환경 변수를 런타임에 읽는다.
#   여기서는 $HOME을 가져와 ~/clearpath/ 경로를 만들 때 사용.
# LaunchConfiguration : DeclareLaunchArgument로 선언된 인자 값을 참조.
#   예: LaunchConfiguration('world') → 실행 시 world 인자 값
# PathJoinSubstitution: 여러 문자열/Substitution을 OS 경로 구분자로 이어 붙인다.
#   예: [pkg_dir, 'launch', 'foo.launch.py'] → '/opt/.../launch/foo.launch.py'
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

# launch_ros.actions.Node: ROS2 노드를 직접 실행할 때 쓰는 액션.
#   package + executable 조합으로 노드를 지정하고 파라미터/리매핑도 여기서 설정한다.
from launch_ros.actions import Node


# ─── generate_launch_description ─────────────────────────────────────────────
# ROS2 launch 시스템 규칙:
#   - 파일 내에 반드시 이 이름(generate_launch_description)의 함수가 있어야 한다.
#   - ros2 launch 명령이 이 함수를 자동으로 호출하고, 반환된 LaunchDescription을
#     처리해 모든 액션을 스케줄링한다.
#   - 인자(args)를 받지 않는 것이 규약이다.
def generate_launch_description():

    # ── 패키지 share 디렉터리 경로 취득 ─────────────────────────────────────
    # 아래 세 변수는 Python str 타입의 절대 경로.
    # PathJoinSubstitution 안에 넣어 launch 파일 경로를 만든다.
    pkg_clearpath_gz = get_package_share_directory('clearpath_gz')
    pkg_clearpath_viz = get_package_share_directory('clearpath_viz')
    pkg_clearpath_nav2_demos = get_package_share_directory('clearpath_nav2_demos')

    # ── 포함할 launch 파일 경로 (Substitution 객체) ──────────────────────────
    # PathJoinSubstitution: 실행 시점에 경로를 결합해 문자열로 확정된다.
    # PythonLaunchDescriptionSource에 넘기면 해당 .py를 import해서 포함시킨다.

    # clearpath_gz/launch/simulation.launch.py
    #   - Gazebo Fortress 시뮬레이터 시작
    #   - 로봇(J100) URDF 스폰
    #   - ros_gz_bridge로 Gazebo ↔ ROS2 토픽 브리지 구성
    simulation_launch = PathJoinSubstitution(
        [pkg_clearpath_gz, 'launch', 'simulation.launch.py'])

    # clearpath_viz/launch/view_navigation.launch.py
    #   - RViz2 실행 (네비게이션 디스플레이 프리셋 포함)
    #   - 로봇 모델, 라이다, 코스트맵, 경로 등 시각화 설정이 미리 들어 있음
    view_navigation_launch = PathJoinSubstitution(
        [pkg_clearpath_viz, 'launch', 'view_navigation.launch.py'])

    # clearpath_nav2_demos/launch/localization.launch.py
    #   - map_server : 저장된 맵(.yaml + .pgm)을 로드해 /map 토픽으로 발행
    #   - amcl       : Adaptive Monte Carlo Localization (파티클 필터)
    #                  맵 위에서 자기 위치를 추정. /j100_0001/initialpose 구독
    localization_launch = PathJoinSubstitution(
        [pkg_clearpath_nav2_demos, 'launch', 'localization.launch.py'])

    # clearpath_nav2_demos/launch/nav2.launch.py
    #   Nav2 풀 스택을 띄운다. 내부적으로 아래 서버들이 함께 뜬다:
    #   - bt_navigator      : Behavior Tree 기반 네비게이션 액션 서버
    #                         (NavigateToPose 골을 받아 BT로 실행)
    #   - planner_server    : 전역 경로 계획 (기본: NavFn / A*)
    #   - controller_server : 로컬 플래너, 실제 cmd_vel 생성 (기본: DWB)
    #   - behavior_server   : 복구 행동 (spin, backup, clear costmap)
    #   - global/local costmap : 장애물 정보를 담는 비용 지도 (두 레이어)
    #   파라미터 기본값: /opt/ros/humble/share/clearpath_nav2_demos/config/j100/nav2.yaml
    nav2_launch = PathJoinSubstitution(
        [pkg_clearpath_nav2_demos, 'launch', 'nav2.launch.py'])

    # ── 공통 경로 치환자 ─────────────────────────────────────────────────────
    # setup_path: ~/clearpath/ 디렉터리
    #   clearpath 패키지가 robot.yaml을 읽어 URDF/world/파라미터를 자동 생성하는 곳.
    #   $HOME 환경변수를 런타임에 읽어 사용자 홈 디렉터리에 독립적으로 동작한다.
    setup_path = [EnvironmentVariable('HOME'), '/clearpath/']

    # map_path: ~/clearpath/maps/my_map.yaml
    #   ros2 run nav2_map_server map_saver_cli 등으로 미리 저장해 둔 맵.
    #   map_server가 이 yaml을 읽어 /map 토픽을 발행한다.
    map_path = [EnvironmentVariable('HOME'), '/clearpath/maps/my_map.yaml']

    # ── launch 인자 선언 ─────────────────────────────────────────────────────
    # 'world': 로드할 Gazebo 월드 이름 (clearpath_gz 패키지 내의 world 파일 이름)
    #   기본값 'warehouse' → clearpath_gz/worlds/warehouse.sdf 가 로드됨
    #   커맨드라인 오버라이드 예:
    #     ros2 launch j100_nav2_bringup bringup.launch.py world:=office
    arg_world = DeclareLaunchArgument(
        'world',
        default_value='warehouse',
        description='Gazebo world to load',
    )

    # ── t=0s : Gazebo 시뮬레이터 ─────────────────────────────────────────────
    # LaunchDescription 시작과 동시에(지연 없이) 실행된다.
    # launch_arguments로 넘기는 파라미터:
    #   'world' → 위에서 선언한 DeclareLaunchArgument 값(기본 'warehouse')
    #             LaunchConfiguration('world')가 런타임에 실제 값으로 치환됨
    # 이 launch가 띄우는 것:
    #   - gzserver / gzclient (Gazebo Fortress)
    #   - spawn_entity: robot.yaml 기반 URDF를 Gazebo에 스폰
    #   - ros_gz_bridge: /scan, /odom, /tf 등 Gazebo ↔ ROS2 브리지
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([simulation_launch]),
        launch_arguments=[
            ('world', LaunchConfiguration('world')),
        ],
    )

    # ── t=0s : RViz 네비게이션 뷰 ────────────────────────────────────────────
    # 시뮬과 동시에(지연 없이) 시작된다.
    # launch_arguments:
    #   'namespace' = 'j100_0001'
    #     → 모든 토픽이 /j100_0001/... 네임스페이스 아래 있으므로
    #       RViz 설정 파일의 토픽 이름이 이 값을 따라야 함
    #   'use_sim_time' = 'true'
    #     → RViz가 /clock 토픽(Gazebo 시뮬 시간)을 기준으로 타임스탬프를 해석
    #       (실제 벽시계 시간이 아니므로 반드시 true로 설정해야 함)
    view_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([view_navigation_launch]),
        launch_arguments=[
            ('namespace', 'j100_0001'),
            ('use_sim_time', 'true'),
        ],
    )

    # ── t=10s : AMCL 로컬라이제이션 ──────────────────────────────────────────
    # TimerAction(period=10.0): launch 시작 10초 후 actions 리스트 실행.
    # 10초를 기다리는 이유:
    #   Gazebo + 로봇 스폰 완료에 시간이 걸린다.
    #   AMCL은 /j100_0001/scan (라이다)과 /j100_0001/odom 을 구독하므로
    #   시뮬레이터가 먼저 완전히 떠야 파티클 필터가 올바르게 초기화됨.
    #
    # launch_arguments:
    #   'setup_path' → ~/clearpath/ (robot.yaml 읽어 네임스페이스/파라미터 자동 생성)
    #   'use_sim_time' = 'true' → AMCL이 /clock을 사용
    #   'map' → ~/clearpath/maps/my_map.yaml (map_server가 로드할 맵)
    localization = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([localization_launch]),
                launch_arguments=[
                    ('setup_path', setup_path),
                    ('use_sim_time', 'true'),
                    ('map', map_path),
                ],
            ),
        ],
    )

    # ── t=15s : Nav2 스택 ────────────────────────────────────────────────────
    # TimerAction(period=15.0): launch 시작 15초 후 실행.
    # 15초를 기다리는 이유:
    #   Nav2는 /map 토픽(map_server 발행)과 AMCL의 /j100_0001/amcl_pose를 구독한다.
    #   AMCL이 10초에 시작되고 초기화까지 추가 시간이 필요하므로
    #   5초 여유를 두고 Nav2를 띄운다.
    #
    # launch_arguments:
    #   'setup_path' → ~/clearpath/ (nav2.yaml 오버라이드, 네임스페이스 설정)
    #   'use_sim_time' = 'true' → 모든 Nav2 서버가 /clock 기준으로 동작
    nav2 = TimerAction(
        period=15.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([nav2_launch]),
                launch_arguments=[
                    ('setup_path', setup_path),
                    ('use_sim_time', 'true'),
                ],
            ),
        ],
    )

    # ── t=25s : initial pose 자동 발행 ───────────────────────────────────────
    # [왜 필요한가?]
    #   AMCL은 처음 시작할 때 로봇 위치를 모른다.
    #   보통은 RViz의 "2D Pose Estimate" 버튼을 사람이 직접 클릭해야 하지만,
    #   이 파일은 자동화를 위해 ExecuteProcess로 ros2 topic pub 을 한 번 실행해
    #   /j100_0001/initialpose 토픽에 메시지를 발행한다.
    #   이 메시지를 받은 AMCL은 파티클 집합을 (x=0, y=0, yaw=0) 주변으로 재배치해
    #   로컬라이제이션을 시작할 수 있다.
    #
    # [토픽 정보]
    #   토픽명   : /j100_0001/initialpose
    #   메시지 타입: geometry_msgs/msg/PoseWithCovarianceStamped
    #
    # [메시지 구조 설명]
    #   geometry_msgs/msg/PoseWithCovarianceStamped
    #   ├── std_msgs/Header header
    #   │     frame_id = 'map'   → 이 pose가 맵 좌표계 기준임을 명시
    #   └── geometry_msgs/PoseWithCovariance pose
    #         ├── geometry_msgs/Pose pose
    #         │     ├── Point position { x=0.0, y=0.0, z=0.0 }
    #         │     │       Gazebo 스폰 위치 = 원점(0,0,0)이므로 일치시킴
    #         │     └── Quaternion orientation { x=0, y=0, z=0, w=1 }
    #         │             yaw=0 (전방이 +X 방향) 쿼터니언 표현
    #         └── float64[36] covariance   ← 6×6 행렬을 행 우선으로 flatten
    #               인덱스 [0]  = σ²_xx  = 0.25  (x 분산, 단위: m²)
    #               인덱스 [7]  = σ²_yy  = 0.25  (y 분산, 단위: m²)
    #               인덱스 [35] = σ²_yaw = 0.0685 (yaw 분산, 단위: rad²)
    #               나머지 33개 = 0.0 (비대각 공분산 없음 → 축 간 독립 가정)
    #               값이 작을수록 "나는 이 위치에 확실히 있다"고 AMCL에 알려줌.
    #               너무 작으면 초기 오차가 있을 때 수렴이 느려지므로
    #               적당한 불확실성(0.25 m², 약 0.27 rad)을 부여함.
    #
    # [--once 플래그]
    #   ros2 topic pub 은 기본적으로 계속 발행하지만 --once 를 쓰면
    #   한 번만 발행하고 프로세스가 종료된다 (latched publisher 효과).
    #   AMCL에 초기 위치를 단 한 번 알려주면 충분하므로 --once 를 사용.
    #
    # [YAML 인라인으로 covariance 36개를 넘기는 이유]
    #   ros2 topic pub 의 메시지 값은 YAML 형식의 문자열로 전달된다.
    #   PoseWithCovarianceStamped의 covariance 필드는 float64[36] 배열이므로
    #   인라인 YAML 리스트로 36개 값을 모두 명시해야 한다.
    initial_pose_yaml = (
        "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, "
        "orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, covariance: "
        "[0.25,0.0,0.0,0.0,0.0,0.0, 0.0,0.25,0.0,0.0,0.0,0.0, "
        "0.0,0.0,0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.0,0.0,0.0, "
        "0.0,0.0,0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.0,0.0,0.0685]}}"
    )

    # TimerAction(period=25.0): launch 시작 25초 후 실행.
    # 25초를 기다리는 이유:
    #   Nav2가 15초에 시작되어 완전히 초기화될 때까지 추가 시간이 필요하다.
    #   Nav2가 /j100_0001/initialpose 구독을 시작하기 전에 발행하면 메시지가 유실됨.
    set_initial_pose = TimerAction(
        period=25.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '--once',
                    '/j100_0001/initialpose',
                    'geometry_msgs/msg/PoseWithCovarianceStamped',
                    initial_pose_yaml,
                ],
                output='screen',
            ),
        ],
    )

    # ── t=30s : Nav 골 UI 노드 ───────────────────────────────────────────────
    # TimerAction(period=30.0): launch 시작 30초 후 실행.
    # 가장 마지막에 띄우는 이유:
    #   initial pose가 발행(25초)되어 AMCL이 로컬라이제이션을 완료한 후
    #   사용자가 UI를 조작해야 의미 있기 때문.
    #   모든 서브시스템이 준비된 후 GUI를 보여주는 것이 UX상 자연스럽다.
    #
    # Node 파라미터:
    #   package    = 'j100_nav2_bringup'   → 이 패키지 자체
    #   executable = 'nav_goal_ui'         → setup.py entry_points에 등록된 스크립트
    #                                        (j100_nav2_bringup/nav_goal_ui.py)
    #   name       = 'nav_goal_ui'         → ROS2 노드 이름 (/nav_goal_ui)
    #   output     = 'screen'              → stdout/stderr를 터미널에 표시
    nav_goal_ui = TimerAction(
        period=30.0,
        actions=[
            Node(
                package='j100_nav2_bringup',
                executable='nav_goal_ui',
                name='nav_goal_ui',
                output='screen',
            ),
        ],
    )

    # ── LaunchDescription 구성 및 반환 ───────────────────────────────────────
    # LaunchDescription에 add_action() 으로 액션을 등록한다.
    # 중요: 실행 순서는 리스트/add_action 순서가 아니라 각 액션의 종류로 결정된다.
    #   - TimerAction이 없는 액션(simulation, view_navigation)은 t=0에 즉시 실행
    #   - TimerAction으로 감싼 액션은 period 값(초)만큼 기다린 후 실행
    #   → 결과적으로 t=0, 10, 15, 25, 30 순으로 컴포넌트가 올라옴
    ld = LaunchDescription()
    ld.add_action(arg_world)        # 인자 선언 (실행 전 파싱 단계에서 처리)
    ld.add_action(simulation)       # t=0: Gazebo 시뮬레이터 + 로봇 스폰
    ld.add_action(view_navigation)  # t=0: RViz 네비게이션 뷰
    ld.add_action(localization)     # t=10s: AMCL 로컬라이제이션
    ld.add_action(nav2)             # t=15s: Nav2 풀 스택
    ld.add_action(set_initial_pose) # t=25s: initial pose 자동 발행
    ld.add_action(nav_goal_ui)      # t=30s: Nav 골 UI
    return ld

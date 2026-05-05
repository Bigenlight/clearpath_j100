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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    pkg_clearpath_gz = get_package_share_directory('clearpath_gz')
    pkg_clearpath_viz = get_package_share_directory('clearpath_viz')
    pkg_clearpath_nav2_demos = get_package_share_directory('clearpath_nav2_demos')

    # Launch paths
    simulation_launch = PathJoinSubstitution(
        [pkg_clearpath_gz, 'launch', 'simulation.launch.py'])

    view_navigation_launch = PathJoinSubstitution(
        [pkg_clearpath_viz, 'launch', 'view_navigation.launch.py'])

    localization_launch = PathJoinSubstitution(
        [pkg_clearpath_nav2_demos, 'launch', 'localization.launch.py'])

    nav2_launch = PathJoinSubstitution(
        [pkg_clearpath_nav2_demos, 'launch', 'nav2.launch.py'])

    # setup_path substitution: $HOME/clearpath/
    setup_path = [EnvironmentVariable('HOME'), '/clearpath/']

    # map path substitution: $HOME/clearpath/maps/my_map.yaml
    map_path = [EnvironmentVariable('HOME'), '/clearpath/maps/my_map.yaml']

    # Declare launch arguments
    arg_world = DeclareLaunchArgument(
        'world',
        default_value='warehouse',
        description='Gazebo world to load',
    )

    # t=0: Gazebo simulation
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([simulation_launch]),
        launch_arguments=[
            ('world', LaunchConfiguration('world')),
        ],
    )

    # t=0: RViz navigation visualisation
    view_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([view_navigation_launch]),
        launch_arguments=[
            ('namespace', 'j100_0001'),
            ('use_sim_time', 'true'),
        ],
    )

    # t=10: Localization (AMCL)
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

    # t=15: Nav2 stack
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

    # t=25: Publish initial pose (origin, identity orientation)
    initial_pose_yaml = (
        "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, "
        "orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, covariance: "
        "[0.25,0.0,0.0,0.0,0.0,0.0, 0.0,0.25,0.0,0.0,0.0,0.0, "
        "0.0,0.0,0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.0,0.0,0.0, "
        "0.0,0.0,0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.0,0.0,0.0685]}}"
    )

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

    # t=30: Qt nav-goal UI node
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

    ld = LaunchDescription()
    ld.add_action(arg_world)
    ld.add_action(simulation)
    ld.add_action(view_navigation)
    ld.add_action(localization)
    ld.add_action(nav2)
    ld.add_action(set_initial_pose)
    ld.add_action(nav_goal_ui)
    return ld

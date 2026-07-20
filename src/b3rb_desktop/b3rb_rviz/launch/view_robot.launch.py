from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, PushRosNamespace


ARGUMENTS = [
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'),
]


def generate_launch_description():

    pkg_b3rb_rviz = get_package_share_directory('b3rb_rviz')

    rviz2_config = PathJoinSubstitution(
        [pkg_b3rb_rviz, 'rviz', 'nav2', 'robot.rviz'])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz2_config,
                   "--ros-args", "--log-level", "fatal"],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ],
        output='log')

    rviz_ld = TimerAction(period=10.0, actions=[rviz_node])

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(rviz_ld)
    return ld

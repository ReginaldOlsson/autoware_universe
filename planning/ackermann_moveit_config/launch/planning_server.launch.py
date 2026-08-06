# Optional standalone robot_state_publisher + debug path publisher.
# Autoware integration uses autoware_moveit_freespace_planner (makePlan), not this launch.

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_description = Command(
        [
            FindExecutable(name='xacro'),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('ackermann_moveit_config'), 'config', 'ackermann.urdf.xacro']
            ),
        ]
    )

    return LaunchDescription(
        [
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'robot_description': robot_description}],
            ),
        ]
    )

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument,ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rosbag_record', default_value = 'false', choices = ['true', 'false'], description = "Enable recording all topics except camera?"),
        # Launch the publisher node
        Node(
            package='tbot_walker',
            executable='walker',
            name='walker_node',
            output='screen'
        ),
            ExecuteProcess(
            condition=IfCondition(LaunchConfiguration('rosbag_record')),
            cmd=['ros2', 'bag', 'record', '-o','walker_bag','-a','-x','depth_cam'],
            shell=True
        )
    ])
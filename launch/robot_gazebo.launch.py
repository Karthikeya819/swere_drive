import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def launch_setup(context):
    six_wheel = context.launch_configurations['six_wheel']
    package_share_dir = get_package_share_directory("swere_drive")
    if six_wheel == 'true':
        xacro_file = "sample_six_wheel.xacro"
    else:
        xacro_file = "sample_four_wheel.xacro"

    urdf = os.path.join(package_share_dir, "urdf", xacro_file)
    gazebo_ros_package = get_package_share_directory('gazebo_ros')
    gazebo_launch_file = os.path.join(gazebo_ros_package, 'launch', 'gazebo.launch.py')

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_file),
        ),
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=["-entity", "my_robot", "-file", urdf]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            arguments=[urdf]
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'six_wheel',
            default_value='false'
        ),
        OpaqueFunction(function=launch_setup)
    ])
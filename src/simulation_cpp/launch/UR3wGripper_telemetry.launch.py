from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # -----------------------------------------------------
    # Paths
    # -----------------------------------------------------
    simulation_pkg = get_package_share_directory('simulation_cpp')
    ur_onrobot_pkg = get_package_share_directory('ur_onrobot_moveit_config')
    ur_driver_pkg = get_package_share_directory('ur_robot_driver')

    # Common Arguments
    robot_ip = '192.168.0.191'
    ur_type = 'ur3e'  # Standardized to ur3e

    # -----------------------------------------------------
    # 1. Launch the Real UR Driver (APT Version)
    # -----------------------------------------------------
    ur_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ur_driver_pkg, 'launch', 'ur_control.launch.py')
        ),
        launch_arguments={
            'ur_type': ur_type,
            'robot_ip': robot_ip,
            'launch_rviz': 'false',
            # We want the driver to handle the hardware interface
            'initial_joint_controller': 'scaled_joint_trajectory_controller',
            'activate_joint_controller': 'true',
        }.items()
    )

    # -----------------------------------------------------
    # 2. Launch MoveIt with OnRobot Gripper
    # -----------------------------------------------------
    ur_onrobot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ur_onrobot_pkg, 'launch', 'ur_onrobot_moveit.launch.py')
        ),
        launch_arguments={
            'ur_type': 'ur3e',
            'robot_ip': '192.168.1.100',
            'use_sim_time': 'false',
            'use_fake_hardware': 'false',
            'onrobot_type': 'rg2',
            'launch_rviz': 'true',
            # ADD THIS LINE BELOW:
            'description_file': 'ur_onrobot.urdf.xacro', 
        }.items()
    )

    # -----------------------------------------------------
    # Optional: custom nodes (example)
    # -----------------------------------------------------
    static_environment_node = Node(
        package='simulation_cpp',
        executable= 'static_environment_node',
        name='static_environment_node',
        output='screen'
    )

    return LaunchDescription([
        ur_driver_launch,
        ur_onrobot_launch,
        static_environment_node,
    ])
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # -----------------------------------------------------
    # Paths
    # -----------------------------------------------------
    ur_onrobot_pkg = get_package_share_directory('ur_onrobot_moveit_config')
    ur_driver_pkg = get_package_share_directory('ur_robot_driver')

    realsense_launch = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch',
        'rs_launch.py'
    )

    # -----------------------------------------------------
    # Common Settings
    # -----------------------------------------------------
    robot_ip = '192.168.0.192'
    ur_type = 'ur3e'

    # -----------------------------------------------------
    # 1. UR Driver (REAL ROBOT)
    # -----------------------------------------------------
    ur_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ur_driver_pkg, 'launch', 'ur_control.launch.py')
        ),
        launch_arguments={
            'ur_type': ur_type,
            'robot_ip': robot_ip,
            'launch_rviz': 'false',
            'initial_joint_controller': 'scaled_joint_trajectory_controller',
            'activate_joint_controller': 'true',
        }.items()
    )

    # -----------------------------------------------------
    # 2. MoveIt + OnRobot Gripper
    # -----------------------------------------------------
    ur_onrobot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ur_onrobot_pkg, 'launch', 'ur_onrobot_moveit.launch.py')
        ),
        launch_arguments={
            'ur_type': ur_type,
            'robot_ip': robot_ip,
            'use_sim_time': 'false',
            'use_fake_hardware': 'false',
            'onrobot_type': 'rg2',
            'launch_rviz': 'true',
            'description_file': 'ur_onrobot.urdf.xacro',
        }.items()
    )

    # -----------------------------------------------------
    # Launch Everything
    # -----------------------------------------------------
    return LaunchDescription([

        # -------- Robot FIRST --------
        ur_driver_launch,
        TimerAction(
            period=2.0,
            actions=[ur_onrobot_launch]
        ),

        # -------- RealSense --------
        TimerAction(
            period=4.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(realsense_launch)
                ),
            ]
        ),

        # -------- Static Workspace --------
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='simulation_cpp',
                    executable='static_environment_node',
                    name='static_environment_node',
                    output='screen'
                ),
            ]
        ),

        # -------- Camera Node --------
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='opencv_camera_cpp',
                    executable='camera_node',
                    name='camera_node',
                    output='screen'
                ),
            ]
        ),

        # -------- Classifier --------
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='rs2_image_processing_package',
                    executable='bottle_classifier_node',
                    name='bottle_classifier',
                    output='screen'
                ),
            ]
        ),

        # -------- GUI --------
        TimerAction(
            period=12.0,
            actions=[
                Node(
                    package='opencv_camera_cpp',
                    executable='mission_control_gui',
                    name='mission_control_gui',
                    output='screen'
                ),
            ]
        ),
    ])
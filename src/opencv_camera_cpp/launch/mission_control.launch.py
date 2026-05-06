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

    realsense_launch = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch',
        'rs_launch.py'
    )

    # -----------------------------------------------------
    # MoveIt + UR + OnRobot Gripper
    # -----------------------------------------------------
    ur_onrobot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ur_onrobot_pkg, 'launch', 'ur_onrobot_moveit.launch.py')
        ),
        launch_arguments={
            'ur_type': 'ur3e',
            'onrobot_type': 'rg2',
            'launch_rviz': 'true',
        }.items()
    )

    return LaunchDescription([

        # -------- Robot stack first --------
        ur_onrobot_launch,

        # -------- Delay RealSense slightly --------
        TimerAction(
            period=2.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(realsense_launch)
                ),
            ]
        ),

        # -------- Static Workspace --------
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='simulation_cpp',
                    executable='static_workspace_node',
                    name='static_workspace',
                    output='screen'
                ),
            ]
        ),

        # -------- Camera Node --------
        TimerAction(
            period=3.0,
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
            period=4.0,
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
            period=4.5,
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
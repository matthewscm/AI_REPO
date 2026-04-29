from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # Realsense launch file path
    realsense_launch = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch',
        'rs_launch.py'
    )

    return LaunchDescription([

        # -------- Camera Node --------
        Node(
            package='opencv_camera_cpp',
            executable='camera_node',
            name='camera_node',
            output='screen'
        ),

        # -------- Realsense --------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch)
        ),

        # -------- Classifier (optional) --------
        Node(
            package='rs2_image_processing_package',
            executable='bottle_classifier_node',
            name='bottle_classifier',
            output='screen'
        ),
        
        # -------- GUI Node --------
        Node(
            package='opencv_camera_cpp',   # <-- CHANGE THIS
            executable='mission_control_gui',
            name='mission_control_gui',
            output='screen'
        ),
    ])
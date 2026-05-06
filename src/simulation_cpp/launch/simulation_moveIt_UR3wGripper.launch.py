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

    # -----------------------------------------------------
    # Launch MoveIt with OnRobot Gripper + custom RViz config
    # -----------------------------------------------------
    ur_onrobot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ur_onrobot_pkg, 'launch', 'ur_onrobot_moveit.launch.py')
        ),
        launch_arguments={
            'ur_type': 'ur3e',
            'onrobot_type': 'rg2',        # <--- Add OnRobot RG2 gripper
            'launch_rviz': 'true',
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
        ur_onrobot_launch,
        static_environment_node,
    ])
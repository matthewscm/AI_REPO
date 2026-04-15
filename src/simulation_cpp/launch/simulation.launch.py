from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Path to the UR MoveIt config package
    ur_moveit_pkg = get_package_share_directory('ur_moveit_config')

    # Include the default MoveIt launch file for UR robots
    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ur_moveit_pkg, 'launch', 'ur_moveit.launch.py')
        ),
        launch_arguments={
            'ur_type': 'ur3',
            'launch_rviz': 'true'
        }.items()
    )


    
    # # -----------------------------------------------
    # # Load Scene Objects (MoveIt PlanningScene)
    # # -----------------------------------------------
    # scene_file = os.path.join(
    #     get_package_share_directory('simulation_cpp'),
    #     "scenes",
    #     "env_wcrates.scene"                 
    # )

    


    # EXAMPLE — Add your own node(s) here
    # my_custom_node = Node(
    #     package='simulation_cpp',
    #     executable='my_node',
    #     name='my_node',
    #     output='screen'
    # )

    
    # # -------------------------------------------
    # # Bottle Grid Spawner Node
    # # -------------------------------------------
    # bottle_spawner_node = Node(
    #     package='simulation_cpp',
    #     executable='bottle_spawner_node',  # <-- change if needed
    #     name='bottle_grid_spawner',
    #     output='screen'
    # )

    return LaunchDescription([
        ur_moveit_launch,
        # scene_loader,
        #bottle_spawner_node,
        # my_custom_node   <--- uncomment when adding nodes
    ])

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
    """
    Resolve the use_fake_hardware argument at launch time so every node
    that needs it gets a consistent value from the single argument above.
    """

    # -----------------------------------------------------------------------
    # Read the single toggle from the command line (or its default)
    # -----------------------------------------------------------------------
    fake_hw = LaunchConfiguration('use_fake_hardware').perform(context)  # 'true' or 'false'
    is_sim  = (fake_hw.lower() == 'true')

    # -----------------------------------------------------------------------
    # Paths
    # -----------------------------------------------------------------------
    ur_onrobot_pkg     = get_package_share_directory('ur_onrobot_moveit_config')
    ur_description_pkg = get_package_share_directory('ur_onrobot_description')
    ur_driver_pkg      = get_package_share_directory('ur_robot_driver')
    realsense_launch   = os.path.join(
        get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py'
    )

    # -----------------------------------------------------------------------
    # Common settings
    # -----------------------------------------------------------------------
    robot_ip = '192.168.0.192'
    ur_type  = 'ur3e'

    # -----------------------------------------------------------------------
    # Robot description (xacro) — consumed by move_to_pose via parameter
    # -----------------------------------------------------------------------
    robot_description_content = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ',
            os.path.join(ur_description_pkg, 'urdf', 'ur_onrobot.urdf.xacro'),
            ' ur_type:=ur3e',
            ' onrobot_type:=rg2',
        ]),
        value_type=str
    )

    # -----------------------------------------------------------------------
    # 1. MoveIt + RViz
    #    • Sim  → use_fake_hardware=true,  no robot_ip required
    #    • Real → use_fake_hardware=false, robot_ip passed through
    # -----------------------------------------------------------------------
    moveit_args = {
        'ur_type':           ur_type,
        'onrobot_type':      'rg2',
        'launch_rviz':       'true',
        'use_fake_hardware': fake_hw,
        'use_sim_time':      'false',
    }
    if not is_sim:
        moveit_args['robot_ip'] = robot_ip

    ur_onrobot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ur_onrobot_pkg, 'launch', 'ur_onrobot_moveit.launch.py')
        ),
        launch_arguments=moveit_args.items()
    )

    # -----------------------------------------------------------------------
    # 2. Static workspace — BOTH sim and real
    #    t=6s — gives MoveIt time to fully start before adding collision objects
    # -----------------------------------------------------------------------
    static_environment = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='simulation_cpp',
                executable='static_environment_node',
                name='static_environment_node',
                output='screen',
            )
        ]
    )

    # -----------------------------------------------------------------------
    # 3. Mission control GUI — BOTH sim and real
    # -----------------------------------------------------------------------
    mission_control_gui = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='opencv_camera_cpp',
                executable='mission_control_gui',
                name='mission_control_gui',
                output='screen',
            )
        ]
    )

    # -----------------------------------------------------------------------
    # 4. Real robot nodes  (REAL ROBOT ONLY)
    #    Timeline:
    #      t=0s   MoveIt starts
    #      t=2s   RealSense camera
    #      t=6s   static environment
    #      t=8s   camera node
    #      t=10s  bottle classifier
    #      t=12s  mission control GUI
    #      t=14s  bottle detector
    #      t=15s  move_to_pose
    # -----------------------------------------------------------------------
    real_robot_nodes = []
    if not is_sim:
        # UR driver
        ur_driver_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ur_driver_pkg, 'launch', 'ur_control.launch.py')
            ),
            launch_arguments={
                'ur_type':                   ur_type,
                'robot_ip':                  robot_ip,
                'launch_rviz':               'false',
                'initial_joint_controller':  'scaled_joint_trajectory_controller',
                'activate_joint_controller': 'true',
                'description_package':       'ur_onrobot_description',
                'description_file':          'urdf/ur_onrobot.urdf.xacro',
            }.items()
        )
        real_robot_nodes.append(ur_driver_launch)

     

        # Camera node
        real_robot_nodes.append(
            TimerAction(
                period=8.0,
                actions=[
                    Node(
                        package='opencv_camera_cpp',
                        executable='camera_node',
                        name='camera_node',
                        output='screen',
                    )
                ]
            )
        )

        # Bottle classifier
        real_robot_nodes.append(
            TimerAction(
                period=10.0,
                actions=[
                    Node(
                        package='rs2_image_processing_package',
                        executable='bottle_classifier_node',
                        name='bottle_classifier',
                        output='screen',
                    )
                ]
            )
        )

        # Bottle detector
        real_robot_nodes.append(
            TimerAction(
                period=14.0,
                actions=[
                    Node(
                        package='bottle_detector',
                        executable='detector_test_node',
                        name='detector_test_node',
                        output='screen',
                    )
                ]
            )
        )

    # -----------------------------------------------------------------------
    # 5. Sim-only nodes  (SIM ONLY)
    # -----------------------------------------------------------------------
    sim_nodes = []
    if is_sim:
        # Bottle spawner
        sim_nodes.append(
            TimerAction(
                period=5.0,
                actions=[
                    Node(
                        package='simulation_cpp',
                        executable='bottle_spawner_node',
                        name='bottle_grid_spawner',
                        output='screen',
                    )
                ]
            )
        )

    # -----------------------------------------------------------------------
    # 6. move_to_pose — BOTH sim and real
    #    Starts at t=15s to ensure MoveIt, controllers, and scene are ready
    # -----------------------------------------------------------------------
    move_to_pose = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='ur3e_motion',
                executable='move_to_pose',
                name='move_to_pose',
                output='screen',
                parameters=[
                    '/home/scribblaboy/RS2_Workspace/kinematics_onrobot.yaml',
                    {
                        'robot_description': robot_description_content,
                        'use_sim_time':      False,
                    },
                ],
            )
        ]
    )
    # RealSense camera — BOTH sim and real
    realsense_node = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(realsense_launch),
                launch_arguments={
                    'align_depth.enable': 'true',
                    'enable_depth':       'true',
                    'enable_color':       'true',
                }.items()
            )
        ]
    )
    # -----------------------------------------------------------------------
    # Return everything
    # -----------------------------------------------------------------------
    return [
        ur_onrobot_launch,
        realsense_node,
        static_environment,
        mission_control_gui,
        *real_robot_nodes,
        *sim_nodes,
        move_to_pose,
    ]


def generate_launch_description():
    return LaunchDescription([
        # -------------------------------------------------------------------
        # THE SINGLE TOGGLE
        #   Sim / testing:   ros2 launch ... use_fake_hardware:=true   (default)
        #   Real robot:      ros2 launch ... use_fake_hardware:=false
        # -------------------------------------------------------------------
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='true',
            description=(
                'true  → simulation / testing mode (fake hardware, bottle spawner)\n'
                'false → real robot mode (UR driver, RealSense, camera pipeline, GUI)'
            )
        ),
        OpaqueFunction(function=launch_setup),
    ])

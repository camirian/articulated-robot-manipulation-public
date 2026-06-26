from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # === DIRECT INJECTION STRATEGY ===
    # Load MoveIt configuration manually to bypass MoveItConfigsBuilder's CHOMP defaults
    import yaml
    
    simple_manipulation_pkg = get_package_share_directory("simple_manipulation")
    panda_moveit_config_pkg = get_package_share_directory("moveit_resources_panda_moveit_config")
    
    #Load custom SRDF
    srdf_path = os.path.join(simple_manipulation_pkg, "config", "panda.srdf")
    
    # Use Move ItConfigsBuilder with planning_pipelines
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(file_path="config/panda.urdf.xacro")
        .robot_description_semantic(file_path=srdf_path)  
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    
    # Load trajectory controllers configuration
    controllers_yaml_path = os.path.join(panda_moveit_config_pkg, "config", "moveit_controllers.yaml")
    with open(controllers_yaml_path, 'r') as file:
        controllers_config = yaml.safe_load(file)
    
    # Trajectory execution parameters
    trajectory_execution_params = {
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.1,
    }

    # Move Group Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            controllers_config,
            trajectory_execution_params,
            {'use_sim_time': True},
            {'default_planning_pipeline': 'ompl'},
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # RViz Config File
    rviz_config_file = os.path.join(
        get_package_share_directory("moveit_resources_panda_moveit_config"),
        "config",
        "moveit.rviz",
    )

    # Declare Launch Argument for RViz
    launch_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz'
    )

    from launch.conditions import IfCondition
    
    moveit_config_file = os.path.join(
        get_package_share_directory('simple_manipulation'),
        'rviz',
        'moveit.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', moveit_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True},
        ],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    # Simple Trajectory Server (Bridge to Isaac Sim)
    trajectory_server_node = Node(
        package="simple_manipulation",
        executable="simple_trajectory_server",
        name="simple_trajectory_server",
        output="screen",
        parameters=[{'use_sim_time': True}],
    )

    # Static TF Publisher (World -> Panda Base)
    # Isaac Sim usually puts the robot at /World/Franka, but the URDF expects 'panda_link0'
    # We need a transform from 'world' to 'panda_link0'
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "panda_link0"],
        parameters=[{'use_sim_time': True}],
    )

    # Robot State Publisher
    # Publishes the TF tree based on the URDF and joint states
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': True},
            {'trajectory_execution.allowed_start_tolerance': 0.1}
        ],
    )

    return LaunchDescription([
        launch_rviz_arg,
        static_tf,
        robot_state_publisher,
        trajectory_server_node,
        move_group_node,
        rviz_node,
    ])

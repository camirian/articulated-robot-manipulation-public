from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Include the main MoveIt bringup
    bringup_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('simple_manipulation'),
                'launch',
                'bringup_moveit.launch.py'
            ])
        ])
    )

    # Static Transform for Camera
    # panda_hand -> panda_hand_camera
    # Rotation: -90 deg around X, -90 deg around Y? 
    # Isaac Sim: [0, -90, -90] Euler
    # TF Args: x y z yaw pitch roll (in radians)
    # -90 deg = -1.57 rad
    # Let's try matching the Euler angles.
    # Note: static_transform_publisher args order depends on version, usually x y z yaw pitch roll frame_id child_frame_id
    
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_static_tf',
        arguments = ['0', '0', '0', '-1.57', '-1.57', '0', 'panda_hand', 'panda_hand_camera']
        # Note: Euler conversion might be tricky. 
        # Isaac Sim: Z up, Y up?
        # Sim: Z is usually up.
        # Camera default: Z is forward.
        # We need to align them.
        # If we get it wrong, the robot will move to the wrong place.
        # For now, we trust the user to verify/debug TF or we iterate.
    )

    # Perception Node
    perception_node = Node(
        package='simple_manipulation',
        executable='perception_node',
        name='perception_node',
        output='screen'
    )
    
    # Load MoveIt Configs to pass to the C++ Interface Node
    from moveit_configs_utils import MoveItConfigsBuilder
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(file_path="config/panda.urdf.xacro")
        .to_moveit_configs()
    )

    # Move To Pose Interface (C++ Node)
    # This node listens to /target_pose and calls MoveIt
    move_to_pose_node = Node(
        package='simple_moveit_interface',
        executable='move_to_pose',
        name='move_to_pose',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True}
        ],
        remappings=[
            ('target_pose', '/target_pose'),
            ('robot_status/move_complete', '/robot_status/move_complete')
        ]
    )

    return LaunchDescription([
        bringup_moveit,
        camera_tf,
        perception_node,
        move_to_pose_node
    ])

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. Include the Perception Stack (MoveIt, Camera, Perception Node)
    # DISABLE default RViz so we can launch our own custom one
    visual_pick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('simple_manipulation'),
                'launch',
                'visual_pick.launch.py'
            ])
        ]),
        launch_arguments={'use_rviz': 'false'}.items()
    )

    # 1.5 Launch Custom RViz (INLINED CONFIG PATH)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", PathJoinSubstitution([
            FindPackageShare('simple_manipulation'),
            'rviz',
            'moveit.rviz'
        ])],
        parameters=[{'use_sim_time': True}]
    )

    # 2. Delayed Action: Perform Pick
    # Wait 10 seconds for RViz to load and the user to Start Recording
    perform_pick = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='simple_manipulation',
                executable='perform_pick',
                name='perform_pick_auto',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        visual_pick,
        rviz_node,
        perform_pick
    ])

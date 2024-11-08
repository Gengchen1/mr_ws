from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    world = LaunchConfiguration('world')
    control_velocity = LaunchConfiguration('control_velocity')
    velocity_noise = LaunchConfiguration('velocity_noise')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution([FindPackageShare('cart_launch'), 'stage_worlds', 'empty.world'])
        ),
        DeclareLaunchArgument('control_velocity', default_value='true'),
        DeclareLaunchArgument('velocity_noise', default_value='0.0'),

        # # Set /use_sim_time parameter
        # SetParameter(name='/use_sim_time', value=True),

        # Node for stage_ros2 with remap
        Node(
            package='stage_ros2',
            executable='stage_ros2',
            name='model',
            output='screen',
            parameters=[{'world_file': world}],
            # remappings=[
            #     ('/odom', '/robot/odom'),
            #     ('/ground_truth', '/robot/base_pose_ground_truth')
            # ]
        ),

        # Conditional node for stage_controller
        Node(
            package='stage_controller',
            executable='stage_controller',
            name='robot',
            output='log',
            condition=IfCondition(control_velocity),
            parameters=[
                {'length': 1.5},
                {'max_steering': 0.5},
                {'max_steering_rate': 1.0},
                {'max_velocity': 18.0},
                {'max_acc': 2.0}
            ],
            # remappings=[
            #     ('steering', '/robot/steering'),
            #     ('velocity', '/robot/veloctity')
            # ]
        ),

        # Conditional node for stage_throttle with parameters and remap
        Node(
            package='stage_controller',
            executable='stage_throttle',
            name='robot',
            output='log',
            condition=UnlessCondition(control_velocity),
            parameters=[
                {'length': 1.5},
                {'max_steering': 0.5},
                {'max_steering_rate': 1.0},
                {'max_velocity': 18.0},
                {'max_throttle': 400},
                {'max_throttle_rate': 800},
                {'max_acc': 2.0},
                {'velocity_noise': velocity_noise},
                {'mass': 500.0},
                {'friction': 20.0},
                {'wind_friction': 0.1},
                {'brake': 15.0},
                {'throttle': 50.0},
                {'exp': 0.4}
            ]
        ),
    ])

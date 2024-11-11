from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution([FindPackageShare('cart_launch'), 'stage_worlds', 'empty.world'])
        ),
        DeclareLaunchArgument('control_velocity', default_value='true'),
        DeclareLaunchArgument('velocity_noise', default_value='0.0'),

        # Node for stage_ros2 with remap
        Node(
            package='stage_ros2',
            executable='stage_ros2',
            name='model',
            output='screen',
            parameters=[{'world_file': LaunchConfiguration('world')}],
            remappings=[
                ('/odom', '/robot/odom'),
                ('/ground_truth', '/robot/base_pose_ground_truth')
            ]
        ),

        # Conditional node for stage_controller
        Node(
            package='stage_controller',
            executable='stage_controller',
            name='robot',
            output='log',
            condition=IfCondition(LaunchConfiguration('control_velocity')),
            parameters=[
                {'length': 1.5},
                {'max_steering': 0.5},
                {'max_steering_rate': 1.0},
                {'max_velocity': 18.0},
                {'max_acc': 2.0}
            ],
            # remappings=[
            #     ('/steering', '/robot/steering'),
            #     ('/velocity', '/robot/velocity')
            # ]
        ),

        # Conditional node for stage_throttle with parameters and remap
        Node(
            package='stage_controller',
            executable='stage_throttle',
            name='robot',
            output='log',
            condition=UnlessCondition(LaunchConfiguration('control_velocity')),
            parameters=[
                {'length': 1.5},
                {'max_steering': 0.5},
                {'max_steering_rate': 1.0},
                {'max_velocity': 18.0},
                {'max_throttle': 400},
                {'max_throttle_rate': 800},
                {'max_acc': 2.0}
            ],
            # remappings=[
            #     ('/steering', '/robot/steering'),
            #     ('/velocity', '/robot/velocity')
            # ]
        ),
    ])
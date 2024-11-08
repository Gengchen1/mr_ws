from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    cart_launch_dir = get_package_share_directory('cart_launch')
    default_world = os.path.join(cart_launch_dir, 'stage_worlds', 'empty.world')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='World file to load'
        ),
        DeclareLaunchArgument(
            'control_velocity',
            default_value='true',
            description='Control velocity'
        ),
        DeclareLaunchArgument(
            'velocity_noise',
            default_value='0.0',
            description='Velocity noise'
        ),
        Node(
            package='stage_ros',
            executable='stageros',
            name='model',
            arguments=[LaunchConfiguration('world')],
            remappings=[
                ('/odom', '/robot/odom'),
                ('/base_pose_ground_truth', '/robot/base_pose_ground_truth')
            ]
        ),
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
            ]
        ),
        Node(
            package='stage_controller',
            executable='stage_throttle',
            name='robot',
            output='log',
            condition=IfCondition(LaunchConfiguration('control_velocity')),
            parameters=[
                {'length': 1.5},
                {'max_steering': 0.5},
                {'max_steering_rate': 1.0},
                {'max_velocity': 18.0},
                {'max_throttle': 400},
                {'max_throttle_rate': 800},
                {'max_acc': 2.0},
                {'velocity_noise': LaunchConfiguration('velocity_noise')},
                {'mass': 500.0},
                {'friction': 20.0},
                {'wind_friction': 0.1},
                {'brake': 15.0},
                {'throttle': 50.0},
                {'exp': 0.4}
            ]
        )
    ])
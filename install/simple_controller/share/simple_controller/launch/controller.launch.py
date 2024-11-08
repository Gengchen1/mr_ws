from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Параметры
    control_velocity = LaunchConfiguration('control_velocity')
    rqt_persp = LaunchConfiguration('rqt_persp')
    velocity_noise = LaunchConfiguration('velocity_noise')

    # Путь к конфигурациям и другим файлам
    simple_controller_dir = get_package_share_directory('simple_controller')
    cart_launch_dir = get_package_share_directory('cart_launch')

    return LaunchDescription([
        # Аргументы, как в ROS 1
        DeclareLaunchArgument(
            'control_velocity', 
            default_value='true'
        ),

        DeclareLaunchArgument(
            'rqt_persp',
            default_value=PathJoinSubstitution([simple_controller_dir, 'rqt_cfg', 'steer_error.perspective'])
        ),
        DeclareLaunchArgument('velocity_noise', default_value='0.0'),

        # Подключение cart_stage.launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([cart_launch_dir, 'launch', 'cart_stage.launch.py'])),
            launch_arguments={
                'control_velocity': control_velocity,
                'velocity_noise': velocity_noise,
            }.items()
        ),

        # Нода для контроллера
        # Node(
        #     package='simple_controller',
        #     executable='controller_node',
        #     name='controller',
        #     output='screen',
        #     parameters=[PathJoinSubstitution([simple_controller_dir, 'launch', 'controller.yaml'])],
        #     remappings=[
        #         ('/controller/simple_controller/ground_truth', '/robot/base_pose_ground_truth'),
        #         ('/controller/simple_controller/odom', '/robot/odom'),
        #         ('steering', '/robot/steering'),
        #     ]
        # ),

        # Нода для rqt
        Node(
            package='simple_controller',
            executable='start_rqt',
            name='rqt',
            arguments=["--perspective-file", LaunchConfiguration("rqt_persp")],
            output='log'
        ),
    ])

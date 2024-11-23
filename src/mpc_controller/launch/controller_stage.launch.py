from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 设置 ROS 日志配置文件环境变量
    # rosconsole_config_file = os.path.join(
    #     get_package_share_directory('mpc_controller'),
    #     'config',
    #     'rosconsole.config'
    # )
    # set_env_var = SetEnvironmentVariable(
    #     name='ROSCONSOLE_CONFIG_FILE',
    #     value=rosconsole_config_file
    # )

    # 包含 cart_stage.launch.py
    cart_stage_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('cart_launch'),
                'launch',
                'cart_stage.launch.py'
            )
        )
    )

    # 启动 mpc_controller 节点
    controller_node = Node(
        package='mpc_controller',
        executable='mpc_controller',
        name='controller',
        output='screen',
        parameters=[os.path.join(
            get_package_share_directory('mpc_controller'),
            'launch',
            'controller.yaml'
        )],
        remappings=[
            ('/controller/mpc_controller/ground_truth', '/robot/base_pose_ground_truth'),
            ('/controller/mpc_controller/odom', '/robot/odom'),
            ('steering', '/robot/steering'),
            ('velocity', '/robot/velocity')
        ]
    )

    # 启动 rviz 节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['--display-config', os.path.join(
            get_package_share_directory('mpc_controller'),
            'rviz',
            'traj.rviz'
        )],
        output='screen'
    )

    return LaunchDescription([
        # set_env_var,
        cart_stage_launch,
        controller_node,
        rviz_node,
    ])
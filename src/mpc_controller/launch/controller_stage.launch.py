from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
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
    mpc_controller_dir = get_package_share_directory('mpc_controller')
    rqt_persp = DeclareLaunchArgument(
        'rqt_persp',
        default_value=mpc_controller_dir + '/rqt/steer_error.perspective'
    )

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
    # 启动 rqt节点
    action_node_start_rqt = Node(
        package='mpc_controller',
        executable='start_rqt',
        name='rqt',
        arguments=["--perspective-file", LaunchConfiguration("rqt_persp")],
        output='screen',  # 改为 screen 以便查看输出
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
            ('/controller/ground_truth', '/robot/base_pose_ground_truth'),
            ('/controller/odom', '/robot/odom'),
            ('steering', '/robot/steering'),
            ('velocity', '/robot/velocity')
        ]
    )

    # 启动 rviz2 节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['--display-config', os.path.join(
            get_package_share_directory('mpc_controller'),
            'rviz',
            'zg.rviz2.rviz'
        )],
        output='screen'
    )

    return LaunchDescription([
        rqt_persp,
        action_node_start_rqt,
        # set_env_var,
        cart_stage_launch,
        controller_node,
        rviz_node,
    ])
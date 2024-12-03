from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.actions import ExecuteProcess
import os

def generate_launch_description():

    mpc_controller_dir = get_package_share_directory('mpc_controller')
    rqt_perspective_path = os.path.join(mpc_controller_dir, 'rqt', 'steer_error.perspective')
    rviz_config_path = os.path.join(mpc_controller_dir, 'rviz', 'traj1.rviz')

    cart_stage_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('cart_launch'),
                'launch',
                'cart_stage.launch.py'
            )
        ),
        launch_arguments={
            'world': os.path.join(get_package_share_directory('cart_launch'), 'stage_worlds', 'empty.world'),
            'control_velocity': 'true',
            'velocity_noise': '0.0'
        }.items()
    )


    # 启动 mpc_controller 节点
    delayed_mpc_node = TimerAction(
        period = 1.0,  # Delay in seconds
        actions = [
            Node(
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
        ]
    )
        
    return LaunchDescription([

        cart_stage_launch,
        delayed_mpc_node,

        # ExecuteProcess to launch RQT
        ExecuteProcess(
            cmd=['rqt', '--perspective-file', rqt_perspective_path],
            name='rqt',
            output='screen',
        ),

        # ExecuteProcess to launch RViz
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config_path],
            name='rviz2',
            output='screen',
        )
    ])
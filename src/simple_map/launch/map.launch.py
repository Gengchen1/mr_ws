import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 声明路径和参数
    cart_launch_dir = get_package_share_directory('cart_launch')
    simple_controller_dir = get_package_share_directory('simple_controller')
    simple_map_dir = get_package_share_directory('simple_map')

    # 声明launch参数
    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(cart_launch_dir, 'stage_worlds', 'mapping.world'),
        description='Path to the stage world file'
    )

    # 加载 cart_stage.launch.py 文件
    cart_stage_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cart_launch_dir, 'launch', 'cart_stage.launch.py')
        ),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    # # simple_controller 节点
    # controller_node = Node(
    #     package='simple_controller',
    #     executable='controller_node',
    #     name='controller',
    #     output='screen',
    #     parameters=[os.path.join(simple_controller_dir, 'launch', 'controller.yaml')],
    #     #remappings=[
    #         #('/controller/simple_controller/ground_truth', '/robot/base_pose_ground_truth'),
    #         #('/controller/simple_controller/odom', '/robot/odom'),
    #         #('steering', '/robot/steering')
    #     #]
    #     remappings=[
    #         ('/ground_truth', '/robot/base_pose_ground_truth'),
    #         ('/odom', '/robot/odom'),
    #         ('steering', '/robot/steering'),
    #     ]
    # )


    # 发布速度指令节点
    velocity_node = Node(
        package='simple_controller',
        executable='velocity_publisher',
        name='vel_node',
        output='screen',
        parameters=[
            {'velocity_value': 2.0, 'publish_rate': 1.0}
    ]
)


    # simple_map 节点
    map_node = Node(
        package='simple_map',
        executable='simple_map_node',
        name='map',
        output='screen',
        remappings=[
            ('scan', '/base_scan')
        ]
    )

    # rviz 节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['--display-config', os.path.join(simple_map_dir, 'rviz', 'map1.rviz')]
    )

    # 返回 LaunchDescription
    return LaunchDescription([
        declare_world_arg,
        cart_stage_launch,
        Node(
            package='simple_controller',
            executable='controller_node',
            name='controller',
            output='screen',
            parameters=[PathJoinSubstitution([simple_controller_dir, 'launch', 'controller.yaml'])],
            remappings=[
                ('/simple_controller/controller/ground_truth', '/robot/base_pose_ground_truth'),
                ('/simple_controller/controller/odom', '/robot/odom'),
                ('/steering', '/robot/steering'),
            ]
        ),
        velocity_node,
        map_node,
        rviz_node
    ])


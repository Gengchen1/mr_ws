import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
  # 参数
  rqt_persp = DeclareLaunchArgument(
    'rqt_persp',
    default_value=PathJoinSubstitution([
      FindPackageShare('velocity_controller'),
      'launch',
      'vel_error.perspective'
    ])
  )
  

  action_node_velocity_controller = launch_ros.actions.Node(
    package='velocity_controller',
    executable='velocity_controller',
    name='throttle_controller',
    output='screen',
    remappings=[
      ('/throttle_controller/odom', '/robot/odom'),
      ('/throttle_controller/velocity', '/robot/velocity'),
      ('/throttle_controller/throttle', '/robot/throttle'),
      ],
  )

  # 启动 rqt节点
  action_node_start_rqt = launch_ros.actions.Node(
    package='simple_controller',
    executable='start_rqt',
    name='rqt',
    arguments=["--perspective-file", LaunchConfiguration("rqt_persp")],
    output='log'
  )

  return launch.LaunchDescription([
    rqt_persp,
    action_node_velocity_controller,
    action_node_start_rqt,
  ])
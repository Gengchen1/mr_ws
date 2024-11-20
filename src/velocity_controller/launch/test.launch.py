import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
  launch_test_time = DeclareLaunchArgument('launch_test_time', default_value="15.0")
  launch_acc = DeclareLaunchArgument('launch_acc', default_value="1.5")
  launch_max_velocity = DeclareLaunchArgument('launch_max_velocity', default_value="5.0")
  launch_noise = DeclareLaunchArgument('launch_noise', default_value="0.0")
  

  action_node_velocity_test = launch_ros.actions.Node(
    package='velocity_controller',
    executable='velocity_test',
    name='vtest',
    remappings=[
      ('/odom', '/robot/odom'),
      ('/velocity', 'robot/velocity')
    ],
    parameters=[
      {
        'test_time': LaunchConfiguration('launch_test_time', default='15.0'),
        'acc': LaunchConfiguration('launch_acc', default='1.5'),
        'max_velocity': LaunchConfiguration('launch_max_velocity', default='5.0'),
      } 
    ]
  )

  return launch.LaunchDescription([
    launch_test_time,
    launch_acc,
    launch_max_velocity,
    launch_noise,
    action_node_velocity_test,
  ])
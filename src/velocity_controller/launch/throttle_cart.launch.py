import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
  launch_noise = DeclareLaunchArgument('noise', default_value="0.0")
  
  # 其他launch文件路径
  controller_stage_launch_path = get_package_share_directory('simple_controller') + '/launch/controller_stage.launch.py'


  action_include_path = launch.actions.IncludeLaunchDescription(
    PythonLaunchDescriptionSource(controller_stage_launch_path),
    launch_arguments={
      'control_velocity': 'false',
      'rqt_persp': '',
      'velocity_noise': LaunchConfiguration('noise')
      }.items()
      )

  return launch.LaunchDescription([
    launch_noise,
    action_include_path,
  ])
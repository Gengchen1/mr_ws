import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
## 1. controller
  # 使用完整路径确保 perspective 文件能被找到
  velocity_controller_dir = get_package_share_directory('velocity_controller')
  rqt_persp = DeclareLaunchArgument(
    'rqt_persp',
    default_value=velocity_controller_dir + '/launch/vel_error.perspective'
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
    output='screen',  # 改为 screen 以便查看输出
    condition=launch.conditions.IfCondition(
        PythonExpression(["'", LaunchConfiguration('rqt_persp'), "' != ''"])
    )
  )
## 2. throttle
  # 参数
  launch_noise = DeclareLaunchArgument('noise', default_value="0.0")
  # 其他launch文件路径
  controller_stage_launch_path = get_package_share_directory('simple_controller') + '/launch/controller_stage.launch.py'

  action_include_path = launch.actions.IncludeLaunchDescription(
    PythonLaunchDescriptionSource(controller_stage_launch_path),
    launch_arguments={
      'control_velocity': 'false',
      'rqt_persp': '',  # 传递空字符串以禁用 steer_error.perspective
      'velocity_noise': LaunchConfiguration('noise')
      }.items()
      )

## 3. test
  launch_test_time = DeclareLaunchArgument('launch_test_time', default_value="15.0")
  launch_acc = DeclareLaunchArgument('launch_acc', default_value="1.5")
  launch_max_velocity = DeclareLaunchArgument('launch_max_velocity', default_value="5.0")
  action_node_velocity_test = launch_ros.actions.Node(
    package='velocity_controller',
    executable='velocity_test',
    name='vtest',
    remappings=[
      ('/vtest/odom', '/robot/odom'),
      ('/vtest/velocity', '/robot/velocity')
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
    # 从上往下启动

    rqt_persp,
    action_node_velocity_controller,
    action_node_start_rqt,

    launch_noise,
    action_include_path,

    launch_test_time,
    launch_acc,
    launch_max_velocity,
    action_node_velocity_test,
  ])






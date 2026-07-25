import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  controller_name = os.environ.get('CONTROLLER', 'yahboom')
  hbot_bringup_dir = get_package_share_directory('hbot_bringup')
  
  # Launch configurations
  params_file = LaunchConfiguration('params_file')
  use_ekf = LaunchConfiguration('use_ekf')
  ekf_params_file = LaunchConfiguration('ekf_params_file')

  # Launch arguments
  declare_params_file_cmd = DeclareLaunchArgument(
    'params_file',
    default_value=os.path.join(hbot_bringup_dir, 'config', 'yahboom_driver_params.yaml'),
    description='Full path to the ROS2 parameters file for the robot driver'
  )

  declare_use_ekf_cmd = DeclareLaunchArgument(
    'use_ekf',
    default_value='True',
    description='Whether to enable robot_localization EKF node'
  )

  declare_ekf_params_file_cmd = DeclareLaunchArgument(
    'ekf_params_file',
    default_value=os.path.join(hbot_bringup_dir, 'config', 'ekf.yaml'),
    description='Full path to the ROS2 parameters file for EKF node'
  )

  # Dynamic driver parameters based on use_ekf
  publish_odom_tf = PythonExpression(['"False" if "', use_ekf, '".lower() in ["true", "1"] else "True"'])
  odom_topic = PythonExpression(['"odom_wheel" if "', use_ekf, '".lower() in ["true", "1"] else "odom"'])
  odom_frame = PythonExpression(['"odom_wheel" if "', use_ekf, '".lower() in ["true", "1"] else "odom"'])

  # 1. Driver launch
  driver_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(
      get_package_share_directory('hbot_driver_' + controller_name),
      'launch',
      'hbot_driver.launch.py'
    )),
    launch_arguments={
      'params_file': params_file,
      'publish_odom_tf': publish_odom_tf,
      'odom_topic': odom_topic,
      'odom_frame': odom_frame,
    }.items()
  )

  # 2. Web node
  web_node = Node(
    package='hbot_web',
    executable='web_node',
    name='hbot_web_node',
    output='screen',
    parameters=[{
      'port': 80,
      'host': '0.0.0.0'
    }]
  )

  # 3. EKF node
  ekf_cmd = Node(
    condition=IfCondition(use_ekf),
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    parameters=[ekf_params_file], 
    remappings=[('/odometry/filtered', '/odom')]
  )

  ld = LaunchDescription()
  ld.add_action(declare_params_file_cmd)
  ld.add_action(declare_use_ekf_cmd)
  ld.add_action(declare_ekf_params_file_cmd)

  ld.add_action(driver_cmd)
  ld.add_action(web_node)
  ld.add_action(ekf_cmd)

  return ld

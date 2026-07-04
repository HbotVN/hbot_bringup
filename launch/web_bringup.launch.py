import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  controller_name = os.environ.get('CONTROLLER', 'yahboom')
  hbot_bringup_dir = get_package_share_directory('hbot_bringup')
  
  # Launch arguments
  params_file = LaunchConfiguration('params_file')
  declare_params_file_cmd = DeclareLaunchArgument(
    'params_file',
    default_value=os.path.join(hbot_bringup_dir, 'config', 'yahboom_driver_params.yaml'),
    description='Full path to the ROS2 parameters file for the robot driver'
  )
  
  # 1. Driver launch
  driver_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(
      get_package_share_directory('hbot_driver_' + controller_name),
      'launch',
      'hbot_driver.launch.py'
    )),
    launch_arguments={'params_file': params_file}.items()
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
  
  ld = LaunchDescription()
  ld.add_action(declare_params_file_cmd)
  ld.add_action(driver_cmd)
  ld.add_action(web_node)
  
  return ld

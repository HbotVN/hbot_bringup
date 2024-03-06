import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition

# Get controller board name
controller_name = os.environ['CONTROLLER']
if controller_name not in ['yahboom']:
  # Log error
  print('Unknown controller: ' + controller_name)
  exit(1)

def generate_launch_description():
  package_name = 'hbot_bringup'


  simulation_mode = LaunchConfiguration('simulation_mode')

  declare_simulation_mode_cmd = DeclareLaunchArgument(
    'simulation_mode',
    default_value='False',
    description='When run as simulation mode'
  )

  # Run driver to control the robot
  driver = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(
      get_package_share_directory('hbot_driver_' + controller_name),
      'launch',
      'hbot_driver.launch.py'
    )),
    condition=IfCondition(PythonExpression(['not ', simulation_mode]))
  )

  ld = LaunchDescription()

  ld.add_action(declare_simulation_mode_cmd)
  ld.add_action(driver)

  return ld
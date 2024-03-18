import os
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch_ros.descriptions import ParameterFile, ParameterValue
from nav2_common.launch import RewrittenYaml, ReplaceString

# Get controller board name
controller_name = os.environ['CONTROLLER']
if controller_name not in ['yahboom']:
  # Log error
  print('Unknown controller: ' + controller_name)
  exit(1)

def generate_launch_description():
  package_name = 'hbot_bringup'
  nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
  slam_toolbox_dir = get_package_share_directory('slam_toolbox')
  slam_launch_file = os.path.join(slam_toolbox_dir, 'launch', 'online_sync_launch.py')

  # Launch arguments
  simulation_mode = LaunchConfiguration('simulation_mode')
  slam = LaunchConfiguration('slam')
  map_yaml_file = LaunchConfiguration('map')
  use_sim_time = LaunchConfiguration('use_sim_time')
  params_file = LaunchConfiguration('params_file')
  slam_params_file = LaunchConfiguration('slam_params_file')
  namespace = LaunchConfiguration('namespace')
  autostart = LaunchConfiguration('autostart')
  use_respawn = LaunchConfiguration('use_respawn')
  log_level = LaunchConfiguration('log_level')

  remappings = [('/tf', 'tf'),
                ('/tf_static', 'tf_static')]

  lifecycle_nodes = ['map_saver']

  param_substitutions = {
    'use_sim_time': use_sim_time,
    'yaml_filename': map_yaml_file
  }

  configured_params = ParameterFile(
    RewrittenYaml(
      source_file=params_file,
      param_rewrites=param_substitutions,
      convert_types=True),
    allow_substs=True)

  stdout_linebuf_envvar = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')

  declare_simulation_mode_cmd = DeclareLaunchArgument(
    'simulation_mode',
    default_value='False',
    description='When run as simulation mode'
  )

  declare_slam_cmd = DeclareLaunchArgument(
    'slam',
    default_value='False',
    description='Whether run SLAM'
  )

  declare_map_yaml_cmd = DeclareLaunchArgument(
    'map',
    default_value=os.path.join(get_package_share_directory('hbot_bringup'), 'maps', 'map.yaml'),
    description='Full path to yaml file with map'
  )

  declare_use_sim_time_cmd = DeclareLaunchArgument(
    'use_sim_time',
    default_value='False',
    description='Use simulation (Gazebo) clock if true'
  )

  declare_params_file_cmd = DeclareLaunchArgument(
    'params_file',
    default_value=os.path.join(get_package_share_directory('hbot_bringup'),'config', 'nav2_params.yaml'),
    description='Full path to the ROS2 parameters file to use for all launched nodes'
  )

  declare_slam_params_file_cmd = DeclareLaunchArgument(
    'slam_params_file',
    default_value=os.path.join(get_package_share_directory('hbot_bringup'),'config', 'slam_params.yaml'),
    description='Full path to the ROS2 parameters file to use for all launched nodes'
  )

  declare_autostart_cmd = DeclareLaunchArgument(
    'autostart',
    default_value='True',
    description='Automatically startup the nav2 stack'
  )

  declare_log_level_cmd = DeclareLaunchArgument(
    'log_level',
    default_value='info',
    description='log_level'
  )

  declare_use_respawn_cmd = DeclareLaunchArgument(
    'use_respawn',
    default_value='False',
    description='Whether to respawn if a node crashes. Applied when composition is disabled.'
  )

  xacro_path = os.path.join(
    get_package_share_directory('hbot_description'),
    'urdf', 'hbot.urdf.xacro')
  urdf_path = os.path.join(
    get_package_share_directory('hbot_description'),
    'urdf', 'hbot.urdf')
  with open(urdf_path, 'r') as infp:
    robot_description = infp.read()

  # Nodes
  hardware_nodes = GroupAction(
    condition=IfCondition(PythonExpression(['not ', simulation_mode])),
    actions = [
      # Run driver to control the robot
      IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
          get_package_share_directory('hbot_driver_' + controller_name),
          'launch',
          'hbot_driver.launch.py'
        )),
      ),
      # Run lidar node
      IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
          get_package_share_directory('lds_006_driver'),
          'launch',
          'lds_006_driver.launch.py'
        )),
      ),
      # Robot description
      # IncludeLaunchDescription(
      #   PythonLaunchDescriptionSource(os.path.join(
      #     get_package_share_directory('hbot_description'),
      #     'launch', 'hbot_description.launch.py'
      #   )),
      #   launch_arguments={'use_sim_time': use_sim_time,
      #                     'rviz': 'false'}.items()
      # )
      Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time,
            'robot_description': robot_description}]
      )
    ]
  )

  # Run SLAM
  slam_cmd_group = GroupAction([
    Node(
      package='nav2_map_server',
      executable='map_saver_server',
      output='screen',
      respawn=use_respawn,
      respawn_delay=2.0,
      arguments=['--ros-args', '--log-level', log_level],
      parameters=[configured_params]
    ),
    Node(
      package='nav2_lifecycle_manager',
      executable='lifecycle_manager',
      name='lifecycle_manager_slam',
      output='screen',
      arguments=['--ros-args', '--log-level', log_level],
      parameters=[{'use_sim_time': use_sim_time},
                  {'autostart': autostart},
                  {'node_names': lifecycle_nodes}]),
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_file),
        launch_arguments={'use_sim_time': use_sim_time,
                          'slam_params_file': slam_params_file}.items())
  ])


  # Run mapping, localization and navigation
  bringup_cmd_group = GroupAction([
    # IncludeLaunchDescription(
    #   PythonLaunchDescriptionSource(os.path.join(launch_dir, 'localization_launch.py')),
    #   condition=IfCondition(PythonExpression(['not ', slam])),
    #   launch_arguments={'namespace': namespace,
    #                     'map': map_yaml_file,
    #                     'use_sim_time': use_sim_time,
    #                     'autostart': autostart,
    #                     'params_file': params_file,
    #                     'use_composition': False,
    #                     'use_respawn': use_respawn,
    #                     'container_name': 'nav2_container'}.items()),

    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(os.path.join(
          get_package_share_directory('nav2_bringup'),
          'launch', 'navigation_launch.py')),
      launch_arguments={'namespace': namespace,
                        'use_sim_time': use_sim_time,
                        'autostart': autostart,
                        'params_file': params_file,
                        'use_composition': False,
                        'use_respawn': use_respawn,
                        'container_name': 'nav2_container'}.items()),
  ])


  ld = LaunchDescription()

  # Set environment variables
  ld.add_action(stdout_linebuf_envvar)

  # Declare launch options
  ld.add_action(declare_simulation_mode_cmd)
  ld.add_action(declare_slam_cmd)
  ld.add_action(declare_map_yaml_cmd)
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_params_file_cmd)
  ld.add_action(declare_slam_params_file_cmd)
  ld.add_action(declare_autostart_cmd)
  ld.add_action(declare_use_respawn_cmd)
  ld.add_action(declare_log_level_cmd)

  # Add the actions to launch all nodes
  ld.add_action(hardware_nodes)
  ld.add_action(slam_cmd_group)
  # ld.add_action(bringup_cmd_group)

  return ld
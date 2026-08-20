import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  controller_name = os.environ.get('CONTROLLER', 'yahboom')
  hbot_bringup_dir = get_package_share_directory('hbot_bringup')
  
  # Launch configurations
  params_file = LaunchConfiguration('params_file')
  use_ekf = LaunchConfiguration('use_ekf')
  ekf_params_file = LaunchConfiguration('ekf_params_file')
  smoother_params_file = LaunchConfiguration('smoother_params_file')
  twist_mux_params_file = LaunchConfiguration('twist_mux_params_file')

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

  declare_smoother_params_file_cmd = DeclareLaunchArgument(
    'smoother_params_file',
    default_value=os.path.join(hbot_bringup_dir, 'config', 'nav2_params.yaml'),
    description='Full path to the ROS2 parameters file for the teleop velocity_smoother node '
                 '(reuses the velocity_smoother block from nav2_params.yaml)'
  )

  declare_twist_mux_params_file_cmd = DeclareLaunchArgument(
    'twist_mux_params_file',
    default_value=os.path.join(hbot_bringup_dir, 'config', 'twist_mux.yaml'),
    description='Full path to the ROS2 parameters file for the twist_mux node that '
                 'arbitrates teleop vs. Nav2 cmd_vel into the final cmd_vel'
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
  # Publishes teleop commands on 'cmd_vel_teleop' (raw, unsmoothed). The
  # velocity_smoother node below rate/accel-limits it onto
  # 'cmd_vel_teleop_smoothed', and twist_mux (2c below) arbitrates that
  # against Nav2's own smoothed output to produce the final 'cmd_vel' the
  # driver actually consumes.
  web_node = Node(
    package='hbot_web',
    executable='web_node',
    name='hbot_web_node',
    output='screen',
    parameters=[{
      'port': 80,
      'host': '0.0.0.0',
      'cmd_vel_topic': 'cmd_vel_teleop'
    }]
  )

  # 2b. Velocity smoother for teleop cmd_vel
  # Reuses the same velocity_smoother tuning (max vel/accel) as the Nav2 stack
  # so teleop and autonomous driving are rate-limited consistently. Runs its
  # own lifecycle manager since base_bringup does not otherwise bring up Nav2.
  #
  # Named "teleop_velocity_smoother" (not "velocity_smoother") because this
  # node runs permanently as part of base_bringup (hbot_web.service), while
  # hbot_bringup.launch.py's own Nav2 stack (started on-demand for SLAM/nav)
  # brings up its own node literally named "velocity_smoother". Two lifecycle
  # nodes sharing one name means their /velocity_smoother/change_state service
  # collides - whichever one answers a transition request meant for the other
  # rejects it instantly (already active/wrong state), which silently aborts
  # the *entire* Nav2 bringup before controller_server/bt_navigator ever
  # activate - so goals published afterwards get no response at all.
  # The RewrittenYaml key_rewrite below renames the "velocity_smoother:" block
  # in nav2_params.yaml to "teleop_velocity_smoother:" so this node still picks
  # up the same tuned values by name-matching, despite the renamed node.
  teleop_smoother_params = ParameterFile(
    RewrittenYaml(
      source_file=smoother_params_file,
      param_rewrites={'use_sim_time': 'False'},
      key_rewrites={'velocity_smoother': 'teleop_velocity_smoother'},
      convert_types=True),
    allow_substs=True)

  velocity_smoother_cmd = Node(
    package='nav2_velocity_smoother',
    executable='velocity_smoother',
    name='teleop_velocity_smoother',
    output='screen',
    parameters=[teleop_smoother_params],
    remappings=[('cmd_vel', 'cmd_vel_teleop'), ('cmd_vel_smoothed', 'cmd_vel_teleop_smoothed')]
  )

  lifecycle_manager_smoother_cmd = Node(
    package='nav2_lifecycle_manager',
    executable='lifecycle_manager',
    name='lifecycle_manager_smoother',
    output='screen',
    parameters=[{
      'use_sim_time': False,
      'autostart': True,
      'node_names': ['teleop_velocity_smoother']
    }]
  )

  # 2c. twist_mux: arbitrates teleop vs. Nav2 cmd_vel into the final 'cmd_vel'.
  # teleop_velocity_smoother (above) outputs 'cmd_vel_teleop_smoothed'.
  # hbot_bringup.launch.py's on-demand Nav2 stack outputs 'cmd_vel_nav_smoothed'
  # (remapped there via a SetRemap around the vendored navigation_launch.py
  # include, so the navigation2 submodule itself is never edited). Before this,
  # both smoothers wrote straight to 'cmd_vel' with no arbitration: since
  # nav2_velocity_smoother's timer keeps firing indefinitely once it has EVER
  # received a command in the session (not just while actively driving), a
  # teleop smoother that had seen any joystick input earlier could silently
  # stomp on Nav2's autonomous driving commands whenever both were alive
  # together. twist_mux.yaml gives teleop strictly higher priority (100 vs 10)
  # so touching the joystick always overrides autonomous driving, with a 0.5s
  # timeout on each source (matching hbot_driver's own cmd_vel watchdog) so an
  # idle source stops blocking the other.
  twist_mux_cmd = Node(
    package='twist_mux',
    executable='twist_mux',
    name='twist_mux',
    output='screen',
    parameters=[twist_mux_params_file, {'use_sim_time': False}],
    remappings=[('cmd_vel_out', 'cmd_vel')]
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
  ld.add_action(declare_smoother_params_file_cmd)
  ld.add_action(declare_twist_mux_params_file_cmd)

  ld.add_action(driver_cmd)
  ld.add_action(web_node)
  ld.add_action(velocity_smoother_cmd)
  ld.add_action(lifecycle_manager_smoother_cmd)
  ld.add_action(twist_mux_cmd)
  ld.add_action(ekf_cmd)

  return ld

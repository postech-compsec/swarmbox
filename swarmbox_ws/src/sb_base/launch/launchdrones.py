import os
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable, SetLaunchConfiguration, TimerAction
from launch.substitutions import LaunchConfiguration
import yaml

from launch.logging import launch_config, get_logger

def get_workspace_root():
    ament_path = os.environ.get('AMENT_PREFIX_PATH', '')
    if ament_path:
        base_path = ament_path.split(':')[0]
        if 'swarmbox_ws' in base_path:
            return base_path.split('swarmbox_ws')[0] + 'swarmbox_ws'
            
    curr_dir = os.getcwd()
    while curr_dir != '/':
        if curr_dir.endswith('swarmbox_ws'):
            return curr_dir
        curr_dir = os.path.dirname(curr_dir)
        
    return None


def launch_setup(context: LaunchContext):
    debug_val = LaunchConfiguration('debug').perform(context).lower() == 'true'

    config_file_path = LaunchConfiguration('config_file').perform(context)
    ws_path = get_workspace_root()
    print(f"DBG: Workspace path: {ws_path}")

    if os.path.isabs(config_file_path):
        full_config_path = config_file_path
    else:
        full_config_path = os.path.join(ws_path, config_file_path)

    try:
        with open(full_config_path, 'r') as config_file:
            config_data = yaml.safe_load(config_file)
        drone_configs = config_data['drone_config']
        package_name = config_data.get('mission_config', {}).get('type', 'unknown')
        get_logger().info(f"Package name: {package_name}")
    except (FileNotFoundError, yaml.YAMLError) as e:
        get_logger().error(f"Error loading configuration file: {e}")
        return []

    nodes_to_launch = []

    log_dir_ss = launch_config.log_dir
    set_env_var = SetEnvironmentVariable('ROS_LOG_DIR', log_dir_ss)
    node_name = 'drone'

    num_drones_val = len(drone_configs['superiors'])
    get_logger().info(f"Number of drone nodes to launch: {num_drones_val}")

    for i in range(num_drones_val):
        superior_id = drone_configs['superiors'].get(i, -1)
        pos_xy = drone_configs['initial_positions'].get(i, [0.0, 0.0])
        relative_pos = [pos_xy[0], pos_xy[1], 0.0]
        namespace = f'drone_{i}'

        get_logger().info(f"{namespace} -> superior: {superior_id}, pos: {relative_pos}")

        ros_args = []
        if debug_val:
            ros_args.append('--log-level')
            ros_args.append(f'{namespace}:=DEBUG')

        drone_node = Node(
            package=package_name,
            executable='idrone',
            name=node_name,
            namespace=namespace,
            output='screen',
            parameters=[{
                'id': i,
                'superior': superior_id,
                'relative_pos': relative_pos
            }],
            ros_arguments=ros_args
        )

        delayed_node = TimerAction(
            period = i * 0.5,
            actions = [drone_node]
        )

        nodes_to_launch.append(
            delayed_node
        )
    return [set_env_var] + nodes_to_launch

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('debug', default_value='false', description='Enable debug mode'),
        DeclareLaunchArgument('config_file', default_value='config.yaml', description='Path to the configuration file'),
        OpaqueFunction(function=launch_setup)
    ])
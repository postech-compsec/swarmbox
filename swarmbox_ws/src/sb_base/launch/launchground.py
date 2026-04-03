import os
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable, RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
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
        mission_config = config_data.get('mission_config', {})
        package_name = config_data.get('mission_config', {}).get('type', 'unknown')
        get_logger().info(f"DBG: Package name: {package_name}")
    except (FileNotFoundError, yaml.YAMLError) as e:
        get_logger().error(f"Error loading configuration file: {e}")
        return []

    log_dir_ss = launch_config.log_dir
    set_env_var = SetEnvironmentVariable('ROS_LOG_DIR', log_dir_ss)
    
    namespace = 'gcs'
    node_name = 'ground'

    ros_args = []
    if debug_val:
        ros_args.append('--log-level')
        ros_args.append(f'{node_name}:=DEBUG')

    mission_type = mission_config.get('type', 'unknown')
    package_name = mission_type
    get_logger().info(f"DBG: Package name: {package_name}")

    sort_val = LaunchConfiguration('sort').perform(context)
    assign_val = LaunchConfiguration('assign').perform(context)

    ground_params = {
        'config_file': full_config_path,
    }

    if sort_val:
        ground_params['str_sort'] = sort_val
    if assign_val:
        ground_params['str_assign'] = assign_val

    
    ground_node = Node(
            package=package_name,
            executable='iground',
            output='screen',
            parameters=[ground_params],
            name=node_name,
            namespace=namespace,
            ros_arguments=ros_args
        )
    
    return [set_env_var, ground_node]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('debug', default_value='false', description='Enable debug mode'),
        DeclareLaunchArgument('config_file', default_value=os.path.join(os.path.dirname(__file__), 'config.yaml'), description='Path to the configuration file'),
        DeclareLaunchArgument('sort', default_value='', description='Sort strategy (0-2) for RQ4'),
        DeclareLaunchArgument('assign', default_value='', description='Assign strategy (0-2) for RQ4'),
        OpaqueFunction(function=launch_setup)
    ])
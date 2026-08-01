import os
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable, RegisterEventHandler, ExecuteProcess, Shutdown
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
    
    target_id_str = LaunchConfiguration('target').perform(context)
    target_id = int(target_id_str) if target_id_str.isdigit() else -1
    delay_ms = float(LaunchConfiguration('delay').perform(context))
    loss_rate = float(LaunchConfiguration('loss').perform(context))

    nodes_to_launch = []

    gcs_remappings = []
    if target_id != -1:
        original = f"/gcs/report"
        faulty   = f"/gcs/report_faulty"
        gcs_remappings.append((original, faulty))
        
        get_logger().info(f"DBG: Remapping topic for gcs: {original} -> {faulty}")

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

    ground_params = {
        'config_file': full_config_path,
    }

    
    ground_node = Node(
            package=package_name,
            executable='iground',
            output='screen',
            parameters=[ground_params],
            name=node_name,
            namespace=namespace,
            ros_arguments=ros_args,
            remappings=gcs_remappings
        )
    
    if target_id != -1:
        # nodes_to_launch.append()
        msg_configs = {# ['report', 'task_cmd', 'heartbeat', 'prox_alert']
            'report': target_id, #f'gcs',
            'heartbeat': target_id, #f'gcs',
            'task_cmd': target_id,
            'prox_alert': target_id
        }
        for topic, target in msg_configs.items():
            fault_injector_node = Node(
                package = 'rq3_faulty',
                executable='faulty_link',
                name=f'faulty_{target_id}_{topic}',
                parameters=[
                    {'target': target,
                    'topic': topic,
                    'delay_ms': delay_ms,
                    'loss_rate': loss_rate}
                ]
            )
            nodes_to_launch.append(fault_injector_node)

    
    return [set_env_var, ground_node] + nodes_to_launch

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('debug', default_value='false', description='Enable debug mode'),
        DeclareLaunchArgument('config_file', default_value=os.path.join(os.path.dirname(__file__), 'config.yaml'), description='Path to the configuration file'),

        DeclareLaunchArgument('target', default_value='-1', description='Target drone ID'),
        DeclareLaunchArgument('delay', default_value='100.0', description='Delay for fault injection'),
        DeclareLaunchArgument('loss', default_value='0.15', description='Packet loss rate for fault injection'),
        OpaqueFunction(function=launch_setup)
    ])
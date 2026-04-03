# ros2 launch sb_base individual_drone.py drone_id:=0 config_file:=./run/evaluation/RQ2/config.yaml
import os
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import yaml
from launch.logging import get_logger

def launch_setup(context: LaunchContext):
    try:
        drone_id = int(LaunchConfiguration('drone_id').perform(context))
    except ValueError:
        get_logger().error("Error: 'drone_id' must be an integer.")
        return []
    
    config_file_path = LaunchConfiguration('config_file').perform(context)
    debug_val = LaunchConfiguration('debug').perform(context).lower() == 'true'

    try:
        with open(config_file_path, 'r') as f:
            config_data = yaml.safe_load(f)
        drone_configs = config_data['drone_config']
        package_name = config_data.get('mission_config', {}).get('type', 'unknown')
    except (FileNotFoundError, yaml.YAMLError, KeyError) as e:
        get_logger().error(f"Error loading or parsing configuration file '{config_file_path}': {e}")
        return []

    try:
        superior_id = drone_configs['superiors'][drone_id]
        pos_xy = drone_configs['initial_positions'][drone_id]
        relative_pos = [float(pos_xy[0]), float(pos_xy[1]), 0.0]
    except (IndexError, KeyError) as e:
        get_logger().error(f"Could not find configuration for drone_id {drone_id} in the YAML file: {e}")
        return []

    namespace = f'drone_{drone_id}'
    node_name = 'drone'

    get_logger().info(
        f"Launching node for {namespace}:\n"
        f"\t- ID: {drone_id}\n"
        f"\t- Superior: {superior_id}\n"
        f"\t- Relative Position: {relative_pos}\n"
        f"\t- Package: {package_name}"
    )
    
    ros_args = []
    if debug_val:
        ros_args.extend(['--log-level', f'{node_name}:DEBUG'])

    drone_node = Node(
        package=package_name,
        executable='idrone',
        name=node_name,
        namespace=namespace,
        output='screen',
        emulate_tty=True,
        parameters=[{
            'id': drone_id,
            'superior': superior_id,
            'relative_pos': relative_pos
        }],
        ros_arguments=ros_args
    )

    return [drone_node]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'drone_id',
            description='The unique ID of the drone to launch.'
        ),
        
        DeclareLaunchArgument(
            'config_file',
            default_value='config.yaml',
            description='Path to the multi-drone configuration YAML file.'
        ),

        DeclareLaunchArgument(
            'debug',
            default_value='false',
            description='Enable debug logging for the node.'
        ),
        
        OpaqueFunction(function=launch_setup)
    ])

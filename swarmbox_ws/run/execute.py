# you can execute desired use case by following execution format:
# .venv/bin/python3 ./run/use_cases/execute.py --config ./run/use_cases/config/<USE_CASE>.yaml 
import argparse
import subprocess
import time
import os
import sys
import yaml

# --- Configuration ---
# get current file's parent directory
current_dir = os.path.dirname(os.path.abspath(__file__))
# unwrap twice
WORKSPACE = current_dir
# unwrap until the WORKSPACE ends with 'swarmbox_ws' and one more time
while not WORKSPACE.endswith('swarmbox_ws'):
    WORKSPACE = os.path.dirname(WORKSPACE)
WORKSPACE = os.path.dirname(WORKSPACE)
print(f"SwarmBox directory: {WORKSPACE}")

PX4_SCRIPT = os.path.join(WORKSPACE, "scripts/px4_swarm_tmux.sh")
SETUP_SCRIPT = os.path.join(WORKSPACE, "swarmbox_ws/install/local_setup.bash") 
AGENT_CMD = ["MicroXRCEAgent", "udp4", "-p", "8888"]

print(f"tmux script: {PX4_SCRIPT}")

# --- Utility Functions ---
def format_time(seconds):
    """Converts seconds into a HH:MM:SS formatted string."""
    if seconds < 0: seconds = 0
    seconds = int(seconds)
    hours, remainder = divmod(seconds, 3600)
    minutes, seconds = divmod(remainder, 60)
    return f"{hours:02}:{minutes:02}:{seconds:02}"

def run_command(command, is_shell_command=False):
    """Executes a given command while hiding its output."""
    display_command = command if is_shell_command else ' '.join(command)
    print(f"  - exec: {display_command}")
    try:
        subprocess.run(command, check=True, cwd=WORKSPACE, shell=is_shell_command, 
                         executable='/bin/bash' if is_shell_command else None,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return True
    except (subprocess.CalledProcessError, FileNotFoundError) as e:
        print(f"  - error: {e}")
        return False

def graceful_shutdown(external_processes):
    """
    Gracefully shuts down all processes, including those in tmux,
    before killing the tmux server.
    """
    print("\n--- Starting Graceful Shutdown ---")

    print("  - Terminating external processes (ROS, Agent)...")
    for p in external_processes:
        if p and p.poll() is None:
            p.terminate()  # SIGTERM
    
    time.sleep(1)
    
    for p in external_processes:
        if p and p.poll() is None:
            p.kill() #SIGKILL

    print("  - Sending Ctrl+C to all tmux panes...")
    try:
        pane_ids_raw = subprocess.check_output(['tmux', 'list-panes', '-a', '-F', '#{pane_id}'], text=True, stderr=subprocess.DEVNULL)
        pane_ids = pane_ids_raw.strip().split('\n')
        
        if pane_ids and pane_ids[0]:
            for pane_id in pane_ids:
                subprocess.run(['tmux', 'send-keys', '-t', pane_id, 'C-c'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        
        print("  - Waiting 3 seconds for tmux processes to terminate...")
        time.sleep(3)

    except (subprocess.CalledProcessError, FileNotFoundError):
        print("  - Tmux server not found or no panes active. Skipping.")
    
    print("  - Killing tmux server.")
    run_command(["tmux", "kill-server"])
    print("--- Graceful Shutdown Complete ---")

# --- Main Logic ---
def main():
    script_start_time = time.time()
    agent_process, ground_process, drones_process = None, None, None

    parser = argparse.ArgumentParser(description="SwarmBox Execution Script")
    parser.add_argument('--config', required=True, type=str, help='Path to the configuration file')
    # parser.add_argument('--num_per_row', type=int, default=4, help='Number of drones per row (default: 4)')
    parser.add_argument('--gz', action='store_true', help='Run with Gazebo (disables headless mode)')
    args = parser.parse_args()

    CONFIG_FILE = args.config
    HEADLESS = "0" if args.gz else "1"
    nod = 0 # initial value
    DRONES_PER_ROW = 0

    try:
        with open(CONFIG_FILE, 'r') as config_file:
            config_data = yaml.safe_load(config_file)
        # read CONFIG_FILE to get the number of drones (mission_config/swarm_size) and drones per row (dpr)
        # Drones per row: calculate from drone_config/initial_positions, get the number of drones that share the same y value with drone 0.
        drone_configs = config_data['drone_config']
        package_name = config_data.get('mission_config', {}).get('type', 'unknown')
        nod = config_data.get('mission_config', {}).get('swarm_size', -1)
        get_drone0_y = drone_configs['initial_positions'].get(0, [0.0, 0.0])[1]
        DRONES_PER_ROW = sum(1 for pos in drone_configs['initial_positions'].values() if pos[1] == get_drone0_y)
        print(f"Package name: {package_name}")
        print(f"Number of drones (nod): {nod}")
        print(f"Drones per row (dpr): {DRONES_PER_ROW}")

    except (FileNotFoundError, yaml.YAMLError) as e:
        print(f"Error reading configuration file: {e}")
        sys.exit(1)



    try:
        print(f"\n{'='*60}")
        print(f"Starting single run for: {CONFIG_FILE}")
        print(f"{'='*60}")

        print("  - Starting MicroXRCE-DDS Agent...")
        agent_process = subprocess.Popen(AGENT_CMD, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        # read CONFIG_FILE has 'formation', 
        if ('formation' in CONFIG_FILE) or ('formation' in os.path.basename(CONFIG_FILE)) or ('rq2' in CONFIG_FILE) or ('rq2' in os.path.basename(CONFIG_FILE)):
            DRONES_PER_ROW = "-1"

        if not run_command(["bash", PX4_SCRIPT, str(nod), str(DRONES_PER_ROW), HEADLESS]):
            return

        ground_ros_cmd = f"ros2 launch sb_base launchground.py config_file:={CONFIG_FILE}"
        drones_ros_cmd = f"ros2 launch sb_base launchdrones.py config_file:={CONFIG_FILE}"
        full_ground_cmd = f"source {SETUP_SCRIPT} && {ground_ros_cmd}"
        full_drones_cmd = f"source {SETUP_SCRIPT} && {drones_ros_cmd}"
        
        print("  - run in background: launchground.py")
        ground_process = subprocess.Popen(['bash', '-c', full_ground_cmd], cwd=WORKSPACE)
        time.sleep(1) 
        
        print("  - run in background: launchdrones.py")
        drones_process = subprocess.Popen(['bash', '-c', full_drones_cmd], cwd=WORKSPACE)

        print("\nwaiting for ros2 processes... (Ctrl+C to interrupt)")
        if ground_process:
            ground_process.wait()
        if drones_process and drones_process.poll() is None:
            drones_process.wait()
        print("  - execution terminated.")

    except KeyboardInterrupt:
        print("\n\nAutomation interrupted by user.")
        sys.exit(130)
    except Exception as e:
        print(f"\n\nAn unexpected error occurred: {e}")
        sys.exit(1)
    finally:
        all_processes = [p for p in [agent_process, ground_process, drones_process] if p]
        graceful_shutdown(all_processes)
    
    print("\n--- Starting Post-Processing ---")
    plotter_env = os.path.join(WORKSPACE, ".venv/bin/python3")
    plotter_script = os.path.join(WORKSPACE, "swarmbox_ws/analysis/plotter.py")
    
    print("  - run: plotter.py")
    subprocess.run([plotter_env, plotter_script, '--config', CONFIG_FILE], cwd=os.path.join(WORKSPACE, "swarmbox_ws"))

    total_script_duration = time.time() - script_start_time
    print(f"\n{'='*60}")
    print(f"Run completed. (Total time: {format_time(total_script_duration)})")
    print(f"{'='*60}")

if __name__ == "__main__":
    if not os.path.exists(SETUP_SCRIPT):
        print(f"Configuration error: SETUP_SCRIPT not found.")
        print(f"  Check the path: {SETUP_SCRIPT}")
    else:
        main()
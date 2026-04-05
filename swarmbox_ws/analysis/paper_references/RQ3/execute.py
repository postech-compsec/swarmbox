# you can execute desired use case by following execution format:
# .venv/bin/python3 ./run/use_cases/execute.py --config ./run/use_cases/config/<USE_CASE>.yaml 

import subprocess
import time
import os
import sys
import json

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
PACKAGE_NAME = "rq3_faulty"
DELAY = 0
LOSS = 0.1
TARGET = 1 # zero-indexed. -1 to disable

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
def main(config_file, target_=-1, delay_=0.0, loss_=0.0):
    script_start_time = time.time()
    agent_process, ground_process, drones_process = None, None, None
    # write parameter file including: config_file, target_, delay_, and loss_ as json in WORKSPACE/sb_log folder
    param_file_path = os.path.join(WORKSPACE, "sb_log", f"params_{int(time.time())}.json")
    os.makedirs(os.path.dirname(param_file_path), exist_ok=True)
    with open(param_file_path, 'w') as f:
        json.dump({
            "config_file": config_file,
            "target": target_,
            "delay": delay_,
            "loss": loss_
        }, f)

    try:
        print(f"\n{'='*60}")
        print(f"Starting single run for package: {PACKAGE_NAME}")
        print(f"{'='*60}")

        print("  - Starting MicroXRCE-DDS Agent...")
        agent_process = subprocess.Popen(AGENT_CMD, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        # read config_file has 'formation', 
        nod = 9
        if ('formation' in config_file) or ('formation' in os.path.basename(config_file)):
            x_r = "-1"
        elif ('delivery' in config_file) or ('delivery' in os.path.basename(config_file)):
            x_r = "5"
            nod = 5
        else:            
            x_r = "3"
        if not run_command(["bash", PX4_SCRIPT, str(nod), x_r, "1"]):
            return

        ground_ros_cmd = f"ros2 launch {PACKAGE_NAME} launchground.py config_file:={config_file} target:={target_} delay:={delay_} loss:={loss_}"
        drones_ros_cmd = f"ros2 launch {PACKAGE_NAME} launchdrones.py config_file:={config_file} target:={target_}"
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
        # read config file from argument "--config <FILE_PATH>"
        CONFIG_FILE = None
        for i, arg in enumerate(sys.argv):
            if arg == "--config" and i + 1 < len(sys.argv):
                CONFIG_FILE = sys.argv[i + 1]
                break
        if not CONFIG_FILE:
            print("Configuration error: Please specify a config file using the --config option.")
        main(CONFIG_FILE, target_=TARGET, delay_=DELAY, loss_=LOSS)
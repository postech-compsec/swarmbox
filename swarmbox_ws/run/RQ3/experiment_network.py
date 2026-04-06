# you can execute desired use case by following execution format:
# .venv/bin/python3 ./run/use_cases/execute.py --config ./run/use_cases/config/<USE_CASE>.yaml 

import argparse
import subprocess
import time
import os
import sys
import json
import numpy as np # For defining parameter ranges
import multiprocessing

# --- Configuration ---
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
# --- ADDED: Mission Timeout Configuration ---
MISSION_TIMEOUT_S = 150 # 2.5 minutes

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
    try:
        subprocess.run(command, check=True, cwd=WORKSPACE, shell=is_shell_command, 
                         executable='/bin/bash' if is_shell_command else None,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return True
    except (subprocess.CalledProcessError, FileNotFoundError) as e:
        print(f"  - error: {e}")
        return False

def graceful_shutdown(all_processes):
    """Gracefully shuts down all managed processes and the tmux server."""
    print("\n--- Starting Graceful Shutdown ---")
    for p in all_processes:
        if p:
            is_running = p.is_alive() if isinstance(p, multiprocessing.Process) else p.poll() is None
            if is_running:
                p.terminate()
    time.sleep(1)
    for p in all_processes:
        if p:
            is_running = p.is_alive() if isinstance(p, multiprocessing.Process) else p.poll() is None
            if is_running:
                p.kill()
    try:
        pane_ids_raw = subprocess.check_output(['tmux', 'list-panes', '-a', '-F', '#{pane_id}'], text=True, stderr=subprocess.DEVNULL)
        pane_ids = pane_ids_raw.strip().split('\n')
        if pane_ids and pane_ids[0]:
            for pane_id in pane_ids:
                subprocess.run(['tmux', 'send-keys', '-t', pane_id, 'C-c'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(3)
    except (subprocess.CalledProcessError, FileNotFoundError):
        pass
    run_command(["tmux", "kill-server"])
    print("--- Graceful Shutdown Complete ---")


# --- Main Logic for a SINGLE RUN ---
def main(config_file, target_=-1, delay_=0.0, loss_=0.0):

    script_start_time = time.time()
    running_processes = []
    
    # Log parameters for this specific run
    param_file_path = os.path.join(WORKSPACE, "swarmbox_ws/sb_log", f"params_{int(time.time())}.json")
    os.makedirs(os.path.dirname(param_file_path), exist_ok=True)
    with open(param_file_path, 'w') as f:
        json.dump({
            "config_file": config_file,
            "target": target_,
            "delay": delay_,
            "loss": loss_
        }, f, indent=4)

    try:
        print(f"\n{'='*60}")
        print(f"Starting Run: target={target_}, delay={delay_}ms, loss={loss_ * 100:.0f}%")
        print(f"{'='*60}")

        print("  - Starting MicroXRCE-DDS Agent...")
        agent_process = subprocess.Popen(AGENT_CMD, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        running_processes.append(agent_process)

        nod = 9
        x_r = "-1"
        
        print("  - Starting PX4 SITL instances...")
        if not run_command(["bash", PX4_SCRIPT, str(nod), x_r, "1"]):
            return

        ground_ros_cmd = f"ros2 launch {PACKAGE_NAME} launchground.py config_file:={config_file} target:={target_} delay:={delay_} loss:={loss_}"
        drones_ros_cmd = f"ros2 launch {PACKAGE_NAME} launchdrones.py config_file:={config_file} target:={target_}"
        full_ground_cmd = f"source {SETUP_SCRIPT} && {ground_ros_cmd}"
        full_drones_cmd = f"source {SETUP_SCRIPT} && {drones_ros_cmd}"
        
        print("  - Launching ground.py and drones.py...")
        ground_process = subprocess.Popen(['bash', '-c', full_ground_cmd], cwd=WORKSPACE, 
                                        #   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
                                          )
        running_processes.append(ground_process)
        time.sleep(1) 
        drones_process = subprocess.Popen(['bash', '-c', full_drones_cmd], cwd=WORKSPACE, 
                                        #   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
                                          )
        running_processes.append(drones_process)

        print("  - Waiting for ROS2 processes to complete...")
        try:
            # If ground process finishes, check if drones process also finished.
            if drones_process and drones_process.poll() is None:
                drones_process.wait(timeout=MISSION_TIMEOUT_S) # Give it a short time to finish up
            print("  - Execution completed normally.")
        except subprocess.TimeoutExpired:
            print(f"\n!!! Mission Timeout ({MISSION_TIMEOUT_S}s) reached. Forcing shutdown. !!!\n")
            # The finally block will handle the cleanup, so we just need to let it fall through.
            pass
        
        if ground_process:
            try:
                ground_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                pass

    except KeyboardInterrupt:
        print("\n\nAutomation interrupted by user.")
        graceful_shutdown(running_processes)
        sys.exit(130)
    except Exception as e:
        print(f"\n\nAn unexpected error occurred: {e}")
        graceful_shutdown(running_processes)
        sys.exit(1)
    finally:
        # This block ensures cleanup happens after every run, including on timeout.
        graceful_shutdown(running_processes)
    
    print("\n--- Starting Post-Processing (Plotter) ---")
    plotter_env = os.path.join(WORKSPACE, ".venv/bin/python3")
    plotter_script = os.path.join(WORKSPACE, "swarmbox_ws/analysis/plotter.py")
    subprocess.run([plotter_env, plotter_script, '--config', config_file], cwd=os.path.join(WORKSPACE, "swarmbox_ws"))

    total_script_duration = time.time() - script_start_time
    print(f"\n{'='*60}")
    print(f"Run Completed. (Total time: {format_time(total_script_duration)})")
    print(f"{'='*60}")


# --- AUTOMATION SCRIPT ENTRY POINT ---
if __name__ == "__main__":
    if not os.path.exists(SETUP_SCRIPT):
        print(f"Configuration error: SETUP_SCRIPT not found: {SETUP_SCRIPT}")
        sys.exit(1)

    parser = argparse.ArgumentParser(description="Network Fault Experiment Automation")
    parser.add_argument('--config', required=True, type=str, help="Path to the configuration file")
    args = parser.parse_args()

    CONFIG_FILE = args.config

    # 2. Define the parameter space for the experiments
    # --- MODIFIED: Iterate through all drones to minimize location bias ---
    TARGET_DRONES = range(9) # Drones 0 through 8
    # TARGET_DRONES = [8]
    
    # Define the values for each experiment
    DELAY_VALUES_MS = [0, 5, 10, 15, 20, 25]
    # DELAY_VALUES_MS = [25]

    LOSS_VALUES_RATIO = np.arange(0.1, 0.6, 0.1) # 0.1, 0.2, 0.3, 0.4, 0.5
    
    automation_start_time = time.time()

    # --- Outer loop to iterate through each target drone ---
    for target_id in TARGET_DRONES:
        print("\n\n" + "*"*80)
        print(f"### STARTING EXPERIMENT SERIES FOR TARGET DRONE: {target_id} ###")
        print("*"*80 + "\n")

        # --- Experiment 1: Communication Delay ---
        print("\n\n" + "#"*80)
        print(f"### DELAY EXPERIMENTS for TARGET DRONE: {target_id} ###")
        print("#"*80 + "\n")
        for delay_val in DELAY_VALUES_MS:
            main(config_file=CONFIG_FILE, target_=target_id, delay_=delay_val, loss_=0.0)
            # --- MODIFIED: Cooldown time changed to 2 seconds ---
            print("--- Cooling down for 2 seconds before next run... ---")
            time.sleep(2)

        # --- Experiment 2: Communication Loss ---
        print("\n\n" + "#"*80)
        print(f"### LOSS EXPERIMENTS for TARGET DRONE: {target_id} ###")
        print("#"*80 + "\n")
        for loss_val in LOSS_VALUES_RATIO:
            loss_val_formatted = float(f"{loss_val:.1f}")
            main(config_file=CONFIG_FILE, target_=target_id, delay_=0.0, loss_=loss_val_formatted)
            # --- MODIFIED: Cooldown time changed to 2 seconds ---
            print("--- Cooling down for 2 seconds before next run... ---")
            time.sleep(2)

    automation_total_time = time.time() - automation_start_time
    print("\n\n" + "#"*80)
    print("### ALL AUTOMATED EXPERIMENTS COMPLETED! ###")
    print(f"Total automation time: {format_time(automation_total_time)}")
    print("#"*80 + "\n")

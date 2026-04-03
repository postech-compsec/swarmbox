import subprocess
import time
import os
import sys

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
CONFIG_DIR = os.path.join(WORKSPACE, "swarmbox_ws/run/RQ4/config")

# --- Utility Functions ---
def format_time(seconds):
    """Converts seconds into a HH:MM:SS formatted string."""
    if seconds < 0: seconds = 0
    seconds = int(seconds)
    hours, remainder = divmod(seconds, 3600)
    minutes, seconds = divmod(remainder, 60)
    return f"{hours:02}:{minutes:02}:{seconds:02}"

# --- Main Logic ---
def run_command(command, is_shell_command=False):
    """Executes a given command while hiding its output."""
    display_command = command if is_shell_command else ' '.join(command)
    print(f"  - exec: {display_command}")
    try:
        subprocess.run(command, check=True, cwd=WORKSPACE, shell=is_shell_command, 
                         executable='/bin/bash' if is_shell_command else None,
                         stdout=subprocess.DEVNULL)
        return True
    except (subprocess.CalledProcessError, FileNotFoundError) as e:
        print(f"  - error: {e}")
        return False

def main():
    # --- Time tracking and progress variables ---
    # range_k = range(1, 3)
    # range_i = range(3)
    # range_j = range(3)
    # total_iterations = len(range_k) * len(range_i) * len(range_j)
    startfrom = 1
    iterate_range = range(startfrom -1, 20*3*3)
    total_iterations = len(iterate_range)
    
    completed_iterations = 0
    total_elapsed_time = 0
    script_start_time = time.time()

    print(f"iterate from {iterate_range[0]} to {iterate_range[-1]}")

    try:
        for iter in iterate_range:

            # for k in range_k:
            #     for i in range_i:
            #         for j in range_j:
            j = iter % 3              # range of (0, 2)
            i = (iter // 3) % 3       # range of (0, 2)
            k = (iter // (3 * 3)) + 1 # range of (1, 20)
            # k = iter % (20 * 3)     
            iter_start_time = time.time()
            completed_iterations += 1
            
            print(f"\n{'='*60}")
            print(f"Starting iteration {completed_iterations + iterate_range[0]}/{total_iterations + iterate_range[0]}: k={k}, i={i}, j={j}")
            print(f"{'='*60}")

            print("  - Starting MicroXRCE-DDS Agent...")
            agent_process = subprocess.Popen(AGENT_CMD, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                
            if not run_command(["bash", PX4_SCRIPT, "5", "5", "1"]):
                run_command(["tmux", "kill-server"])
                continue
            

            config_file = os.path.join(CONFIG_DIR, f"rq4_{k}.yaml")
            ground_ros_cmd = f"ros2 launch sb_base launchground.py sort:={i} assign:={j} config_file:={config_file}"
            drones_ros_cmd = f"ros2 launch sb_base launchdrones.py config_file:={config_file}"
            full_ground_cmd = f"source {SETUP_SCRIPT} && {ground_ros_cmd}"
            full_drones_cmd = f"source {SETUP_SCRIPT} && {drones_ros_cmd}"
            
            ground_process, drones_process = None, None
            try:
                print("  - run in background: launchground.py")
                ground_process = subprocess.Popen(['bash', '-c', full_ground_cmd], cwd=WORKSPACE, 
                                                #   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
                                                    )
                time.sleep(1) # Uncomment if needed
                
                print("  - run in background: launchdrones.py")
                drones_process = subprocess.Popen(['bash', '-c', full_drones_cmd], cwd=WORKSPACE, 
                                                #   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
                                                    )

                print("\nwaiting for ros2 processes... (Ctrl+C to interrupt)")
                ground_process.wait()
                drones_process.wait()
                print("  - execution terminated.")

            except Exception as e:
                print(f"  - ros2 launch error: {e}")
                if ground_process: ground_process.kill()
                if drones_process: drones_process.kill()
                print("  - killing ros2 processes.")
            
            finally:
                print("\ncleaning up MicroXRCE-DDS Agent...")
                if agent_process:
                    agent_process.kill()
                    agent_process.wait()
                print("\ncleaning up tmux server...")
                run_command(["tmux", "kill-server"])

            print("\n--- Starting Post-Processing ---")
            plotter_env = os.path.join(WORKSPACE, ".venv/bin/python3")
            plotter_script = os.path.join(WORKSPACE, "swarmbox_ws/analysis/plotter.py")
            strategy_arg = f"{i},{j}"
            subprocess.run([plotter_env, plotter_script, '--config', config_file, '--strategy', strategy_arg], cwd=os.path.join(WORKSPACE, "swarmbox_ws"))

            # --- Time calculation and ETC display ---
            iter_duration = time.time() - iter_start_time
            total_elapsed_time += iter_duration
            average_time = total_elapsed_time / completed_iterations
            remaining_iterations = total_iterations - completed_iterations
            etc_seconds = average_time * remaining_iterations

            print(f"\n{'-'*25} Results {'-'*25}")
            print(f"  - Iteration time: {format_time(iter_duration)}")
            if remaining_iterations > 0:
                print(f"  - Estimated time remaining (ETC): {format_time(etc_seconds)}")
            print(f"{'-'*59}")

    except KeyboardInterrupt:
        print("\n\nAutomation interrupted by user.")
        run_command(["tmux", "kill-server"], is_shell_command=True)
        print(f"killed at iteration #{completed_iterations + iterate_range[0]}")
        sys.exit(130)
    except Exception as e:
        print(f"\n\nAn unexpected error occurred: {e}")
        run_command(["tmux", "kill-server"], is_shell_command=True)
        sys.exit(1)

    total_script_duration = time.time() - script_start_time
    print(f"\n{'='*60}")
    print(f"Automation completed successfully. (Total time: {format_time(total_script_duration)})")
    print(f"{'='*60}")

if __name__ == "__main__":
    if not os.path.exists(SETUP_SCRIPT):
        print(f"Configuration error: SETUP_SCRIPT not found.")
        print(f"  Check the path: {SETUP_SCRIPT}")
    else:
        main()
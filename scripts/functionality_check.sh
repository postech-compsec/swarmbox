#!/bin/bash

set -e

trap "echo -e '\nShutting down SwarmBox and cleaning up...'; pkill -f px4; pkill -f gzserver; pkill -f gzclient; pkill -f ros2; exit" SIGINT SIGTERM

echo "=================================================="
echo " SwarmBox Functionality Check"
echo "=================================================="

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"
cd "$WORKSPACE_ROOT"

echo "▶ [1/4] Activating Virtual Environment..."
source .venv/bin/activate

echo "▶ [2/4] Checking PX4-Autopilot Build..."
cd PX4-Autopilot 
make px4_sitl
cd "$WORKSPACE_ROOT"

echo "▶ [3/4] Checking SwarmBox Workspace Build..."
cd swarmbox_ws 
colcon build

echo "▶ [4/4] Launching Formation Flight Simulation..."
source install/setup.bash 
python3 ./run/execute.py --config ./run/RQ2/config/rq2_sim2real.yaml

echo "=================================================="
echo " Functionality Check Completed!"
echo "=================================================="
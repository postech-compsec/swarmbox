#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"
cd "$WORKSPACE_ROOT"

# install tmux
if ! command -v tmux &> /dev/null; then
    echo "tmux could not be found. Installing tmux..."
    sudo apt-get update
    sudo apt-get install -y tmux
else
    echo "tmux is already installed."
fi

# install cloc if not installed
if ! command -v cloc &> /dev/null; then
    echo "cloc could not be found. Installing cloc..."
    sudo apt update
    sudo apt install -y cloc
else
    echo "cloc is already installed."
fi

echo "Initializing submodules..."
git submodule update --init --recursive

echo "Setting up Python environment..."
PY_VER=$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')

if ! python3 -m venv -h &> /dev/null; then
    echo "venv module is missing for Python ${PY_VER}. Installing..."
    sudo apt-get update -yq
    sudo apt-get install -yq "python${PY_VER}-venv"
fi

python3 -m venv .venv
source .venv/bin/activate

echo "Installing Python dependencies..."
pip install -r ./scripts/requirements.txt
pip install -r ./PX4-Autopilot/Tools/setup/requirements.txt

echo "Setting up PX4-Autopilot..."
(cd PX4-Autopilot && bash Tools/setup/ubuntu.sh)

# IF needed, Install ROS 2 dependencies. mostly it's not needed.
# source /opt/ros/humble/setup.bash
# sudo rosdep init 2>/dev/null || true
# rosdep update
# rosdep install --from-paths swarmbox_ws/src -y --ignore-src

echo "Building PX4-Autopilot..."
(cd PX4-Autopilot && make px4_sitl)

echo "Building ROS 2 workspace..."
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    ROS_DISTRO="jazzy"
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    ROS_DISTRO="humble"
else
    echo "❌ Error: Neither ROS 2 Jazzy nor Humble setup file was found in /opt/ros/"
    echo "Please ensure ROS 2 is installed correctly."
    exit 1
fi

source /opt/ros/${ROS_DISTRO}/setup.bash
(cd swarmbox_ws && colcon build)

echo "We recommend to add the following line to your .bashrc or .zshrc to source the ROS 2 workspace automatically:"
echo "source /opt/ros/${ROS_DISTRO}/setup.bash (or .zsh)"

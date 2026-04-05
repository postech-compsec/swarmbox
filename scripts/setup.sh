# !#/bin/bash

echo "Initializing submodules..."
git submodule update --init --recursive

echo "Setting up Python environment..."
python3 -m venv .venv
source .venv/bin/activate

echo "Installing Python dependencies..."
pip install -r ./scripts/requirements.txt
pip install -r ./PX4-Autopilot/Tools/setup/requirements.txt

echo "Installing ROS 2 dependencies..."
source /opt/ros/humble/setup.bash
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths swarmbox_ws/src -y --ignore-src

echo "Building ROS 2 workspace..."
cd swarmbox_ws
colcon build

echo "We recommend to add the following line to your .bashrc or .zshrc to source the ROS 2 workspace automatically:"
echo "source /opt/ros/humble/setup.bash (or .zsh)"

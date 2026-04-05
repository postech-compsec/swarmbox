#!/bin/bash

echo "========== Cleanup =========="

# 1. Remove logs in analysis/data/ but keep .gitignore
echo "[-] Cleaning analysis data logs..."
if [ -d "swarmbox_ws/analysis/data" ]; then
    find swarmbox_ws/analysis/data/ -mindepth 1 ! -name '.gitignore' -exec rm -rf {} + 2>/dev/null || true
else
    echo "    ↳ Directory not found, skipping."
fi

# 2. Remove CSV files in sb_log
echo "[-] Cleaning sb_log CSV files..."
if [ -d "swarmbox_ws/sb_log" ]; then
    rm -f swarmbox_ws/sb_log/*.csv
else
    echo "    ↳ Directory not found, skipping."
fi

# 3. Remove ROS 2 workspace build directories
echo "[-] Removing ROS 2 build, log, and install directories..."
rm -rf swarmbox_ws/build/ swarmbox_ws/log/ swarmbox_ws/install/

# 4. Clean PX4-Autopilot (make distclean)
echo "[-] Running 'make distclean' on PX4-Autopilot..."
if [ -d "PX4-Autopilot" ]; then
    (cd PX4-Autopilot && make distclean)
else
    echo "    ⚠️ PX4-Autopilot directory not found! Skipping..."
fi

echo "Cleanup Completed."
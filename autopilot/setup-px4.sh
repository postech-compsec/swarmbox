#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

export PX4_VERSION="release/1.17"

if [ -d "PX4-Autopilot" ] && [ -n "$(ls -A PX4-Autopilot 2>/dev/null)" ]; then
    echo "PX4-Autopilot already exists at $SCRIPT_DIR/PX4-Autopilot, skipping clone."
    echo "Remove this directory first if you want to re-clone and re-apply patches."
else
    git clone https://github.com/PX4/PX4-Autopilot.git -b "$PX4_VERSION" --recursive

    # apply patches
    echo "Creating swarmbox airframe.."
    (cd PX4-Autopilot && git apply ../PX4-patches/airframe.patch)

    echo "Applying dds topic patch.."
    (cd PX4-Autopilot && git apply ../PX4-patches/dds_topics.patch)
fi

export PX4_HOME="$SCRIPT_DIR/PX4-Autopilot"

echo "Installing PX4-Autopilot Python dependencies..."
# install inside venv!
../.venv/bin/pip install -r ./PX4-Autopilot/Tools/setup/requirements.txt

echo "Installing PX4-Autopilot system dependencies..."
(cd PX4-Autopilot && bash Tools/setup/ubuntu.sh)


echo "Building PX4-Autopilot..."
(cd PX4-Autopilot && make px4_sitl)

echo "PX4-Autopilot setup complete at $PX4_HOME"


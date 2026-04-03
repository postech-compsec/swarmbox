#!/bin/bash
# CONFIG: change px4 path if needed.
PX4_PATH="PX4-Autopilot/build/px4_sitl_default/bin/px4"

# usage: ./px4_swarm_tmux.sh <number_of_drones> <num_of_drones_on_one_row> <headless_flag>
# automates px4 simulation multi drone runs

pkill ruby 2>/dev/null
pkill -x px4 2>/dev/null
pkill -9 -f "gz sim" 2>/dev/null

drones=$1
drones=$((drones - 1))
x_r=$2
headless=${3:-0} 

SESSION_NAME="swarmbox"
tmux new-session -d -s $SESSION_NAME 2>/dev/null || tmux attach -t $SESSION_NAME

if [ "$headless" -eq 0 ]; then
    hdls=" "
else
    hdls=" HEADLESS=1"
fi

for ((n=0; n<=drones; n++)); do
    # airview, delivery
    north=$((n / x_r * 5))
    east=$((n % x_r * 5))

    # if x_r is -1, its formation mode:
    if (( x_r == -1 )); then
        # formation mode
        fl=$(((n + 1) / 2))
        east=$((-3 * fl))
        north=$((-1 * east * ((-1)**n)))
    fi

    command="PX4_SYS_AUTOSTART=4001$hdls PX4_SIM_MODEL=gz_x500 PX4_GZ_MODEL_POSE=\"$north,$east,0,0,0,0\" $PX4_PATH -i $n"
    tmux send-keys -t $SESSION_NAME "$command" C-m
    tmux split-window -v
    tmux select-layout tiled
    
    if [ "$n" -lt 1 ]; then
        sleep 3
    else
        sleep 1
    fi

    echo "Drone $n started: $north, $east"
done

# tmux attach -t $SESSION_NAME

# you can end the session by following command
# tmux kill-session -t swarmbox
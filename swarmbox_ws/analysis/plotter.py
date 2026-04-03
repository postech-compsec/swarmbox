import pandas as pd
import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import pyulog, os, re, datetime, json, yaml, sys
import shutil
import argparse
from pathlib import Path

### >> CONFIG: set path configurations as needed << ###
file_path = os.path.abspath(__file__)
# project and px4 are at ../.. from this file
repo_dir = os.path.dirname(os.path.dirname(os.path.dirname(file_path)))
project_path = os.path.join(repo_dir, 'swarmbox_ws/')
px4_path = os.path.join(repo_dir, 'PX4-Autopilot/')
#######################################################

ulg_files = []
linetypes = ['solid', 'dash', 'dot', 'dashdot', 'longdash', 'longdashdot']
colors =  ["#B25D12", "#0E94A3",'#D62728',  '#9467BD', '#2CA02C',  "#7262FF", "#8D8D1A", '#68372D']
plot_markers = False

# px4 and roslog path configuration
px4_path += 'build/px4_sitl_default/rootfs'
# now get ROS logs and find the timestamp diff
# ros_log_path = $ROS_LOG_DIR if $ROS_LOG_DIR exists, if not: $ROS_HOME/log if $ROS_HOME exists, else: '~/.ros/log'
ros_log_path = os.getenv('ROS_LOG_DIR')
if not ros_log_path:
    ros_home = os.getenv('ROS_HOME')
    if ros_home:
        ros_log_path = os.path.join(ros_home, 'log')
    else:
        ros_log_path = '~/.ros/log'
ros_log_dir = os.path.expanduser(ros_log_path)

# read latest 2 folders
ros_folders = sorted([os.path.join(ros_log_dir, f) for f in os.listdir(ros_log_dir) if os.path.isdir(os.path.join(ros_log_dir, f))], key=os.path.getctime, reverse=True)[:2]
ros_files = []
for folder in ros_folders:
    # read all .log files in the folder and add to ros_files
    files = [os.path.join(folder, f) for f in os.listdir(folder) if f.endswith('.log')]
    ros_files.extend(files)
# sort ros_files by creation time, and take the latest droneno files
# ros_files = sorted([os.path.join(ros_log_dir, f) for f in os.listdir(ros_log_dir) if f.endswith('.log')], key=os.path.getctime, reverse=True)[:droneno+1]
# remove ...launch.log file and make them as launch_files
launch_files = [f for f in ros_files if 'launch.log' in f]
ros_files = [f for f in ros_files if 'launch.log' not in f]
ros_timegaps = dict()
ros_reference_ts = 0
ground_init_time = 0

# # find package name from launch_files[0] "DBG: Package name: " and save as package_name
# if launch_files:
#     with open(launch_files[0], 'r') as f:
#         # FIXME:
#         for line in f:
#             if "DBG: Package name: " in line:
#                 package_name = line.split("DBG: Package name: ")[1].strip()
#                 break

# print(f"Package name: {package_name}")

# initialize variables
init_pos = {}
config_path = ''
execinfo = ''
DRONES_PER_ROW = 0


parser = argparse.ArgumentParser(description="SwarmBox Plotter")
parser.add_argument('--config', required=True, type=str, help="Path to the configuration file")
parser.add_argument('--plot_markers', action='store_true', help="Whether to plot setpoint markers")
parser.add_argument('--strategy', required=False, type=str, help="Strategy in format 'sort_strategy,assign_strategy'")
args = parser.parse_args()

config_path = Path(args.config).resolve()
plot_markers = args.plot_markers

print(f"Using config path: {config_path}")

execinfo = config_path.stem 

# RQ3: save discrepancies
rq3 = execinfo.startswith('rq3')

# RQ4: strategy arguments
strategy = args.strategy
sort_strategy, assign_strategy = '', ''
if strategy:
    sort_strategy, assign_strategy = strategy.split(',')
    print(f"Using sort strategy: {sort_strategy}")
    print(f"Using assign strategy: {assign_strategy}")
    execinfo += f"_s{sort_strategy}_a{assign_strategy}"


# # get arguments from execution
# if len(sys.argv) > 1:
#     for i in range(len(sys.argv)):
#         if sys.argv[i] == '--config' and i + 1 < len(sys.argv):
#             config_path = sys.argv[i + 1]
#             print(f"Using config path: {config_path}")
#             # if path is relative, make it absolute based on current file path
#             continue
#     if config_path.endswith('.yaml'):
#         execinfo = f"{config_path.split('/')[-1].replace('.yaml', '')}"
#     else:
#         execinfo = f"{config_path.split('/')[-1]}"
#     print(f"Directory postfix: {execinfo}")
# else:
#     config_path = os.path.expanduser(config_path + f'/{package_name}.yaml')
#     print("Please provide --config <config_path> argument.")
#     exit()

print(f"Directory postfix: {execinfo}")

save_pref_path = os.path.expanduser(project_path + 'analysis/data/')
save_pref_path += str(datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))+f'_{execinfo}'+"/"
if not os.path.exists(save_pref_path):
    os.makedirs(save_pref_path)
    print(f"Created directory: {save_pref_path}")

# read config file
try:
    with open(config_path, 'r') as f:
        config_data = yaml.safe_load(f)
    init_pos = config_data.get('drone_config', {}).get('initial_positions', {})
    droneno = len(init_pos)
    DRONES_PER_ROW = int(np.ceil(np.sqrt(droneno)))
except (FileNotFoundError, yaml.YAMLError) as e:
    print(f"Error loading configuration file: {e}")
    # pos = [0, 0]
droneno = len(init_pos)

# read folder list and select latest one
for i in range(0, droneno):
    # if (file_nth == 0):
    px4_log_dir = px4_path + f'/{i}/log'
    px4_log_dir = os.path.expanduser(px4_log_dir)
    if os.path.exists(px4_log_dir):
        files = os.listdir(px4_log_dir)
        if files:
            latest_folder = max([os.path.join(px4_log_dir, f) for f in files], key=os.path.getctime)
            # ulg_files.append(latest_folder)
            # now select latest file in the folder
            files = os.listdir(latest_folder)
            if files:
                latest_file = max([os.path.join(latest_folder, f) for f in files], key=os.path.getctime)
                ulg_files.append(latest_file)
            else:
                print(f"No files found in {latest_folder}")
    # else:
        # if user have specified to read nth file

# for rq3 gps drift detection
if rq3: estimator_stats = {}

# from those ulg files, read ['vehicle_local_position_0', 'trajectory_setpoint_0', 'vehicle_attitude_0']
ulog_data = []
ulog_msgs = {}
for file in ulg_files:
    ulog = pyulog.ULog(file)
    data = {}
    for topic in ['vehicle_local_position', 'trajectory_setpoint', 'vehicle_attitude', 'vehicle_status', 'vehicle_land_detected']:
        data[topic] = ulog.get_dataset(topic).data
    ulog_data.append(data)
    ulog_msgs[int(file.split("/")[-4])] = ulog.logged_messages
    drone_id = int(file.split("/")[-4])
    if rq3: estimator_stats[drone_id] = pd.DataFrame(ulog.get_dataset('estimator_status').data)[['timestamp', 'reset_count_pos_ne']]

if rq3:
    reset_dfs = [] 
    
    for drone_id, est_df in estimator_stats.items():
        est_df['reset_count_pos_ne'] = est_df['reset_count_pos_ne'].where(est_df['reset_count_pos_ne'].diff() != 0, other=pd.NA)
        est_df.dropna(inplace=True)
        est_df['drone_id'] = drone_id
        
        reset_dfs.append(est_df) 

    if reset_dfs:
        estimator_resets = pd.concat(reset_dfs, ignore_index=True)
    else:
        estimator_resets = pd.DataFrame(columns=['drone_id', 'timestamp', 'reset_count_pos_ne'])
        
    estimator_resets.to_csv(save_pref_path+'fordrift_estimator_resets.csv', index=False)

sb_states = {
    'ABORTION.': 'abort',
    'ICEBREAK.': 'icebreak', 
    'PREPARATION.' : 'preparation', 
    'MISSION EXECUTION.': 'execution', 
    'LANDING (completion).': 'landing', 
    'TERMINATION.': 'terminate'
}
# read logs with "Stage Update: set to "
ros_states = []

states_df = pd.DataFrame(columns=['timestamp', 'identity', 'state'])
markers = pd.DataFrame(columns=['timestamp', 'identity', 'marker', 'message'])
exec_completion = {}

for file in ros_files:
    # read the line that contains "PX4 timestamp: "
    identity = ''
    with open(file, 'r') as f:
        # we can get ground identity from file name
        if 'iground' in file.lower():
            identity = 'ground'
            # print(f"Ground Control Station log file: {file}")
        lines = f.readlines()
        for line in lines:
            # first get identity
            if "Ground node initialized." in line:
                # make this timestamp as reference
                ground_init_time = int(float(re.findall(r'\[(.*?)\]', line)[1]) * 1000000)
                print(f"ground initialized at: {ground_init_time}")
            if "Drone node" in line:
                id_no = line.split("Drone node ")[1].split(" initialized")[0]
                identity = f"drone_{id_no}"

            # now let's find timestamp
            if "PX4 timestamp: " in line:
                # extract the timestamp
                parsed = line.split("PX4 timestamp: ")
                px4_timestamp = int(parsed[1].strip())
                ros_timestamp = int(float(re.findall(r'\[(.*?)\]', line)[1]) * 1000000)
                if (ros_reference_ts < ros_timestamp):
                    ros_reference_ts = ros_timestamp
                drone_id = int(parsed[0][37:].split('.drone')[0])
                identity = f"drone_{drone_id}"
                # px4_timestamp = int(line.split("PX4 timestamp: ")[1].strip())
                # ros_timestamp = 
                ros_timegaps[drone_id] = ros_timestamp - px4_timestamp
                print(f"drone_id: {drone_id}, PX4 timestamp: {px4_timestamp}, ROS timestamp: {ros_timestamp}, Time gap: {ros_timegaps[drone_id]}")

            # read state changes
            if "initialized" in line:
                state = 'icebreak'
                timestamp = int(float(re.findall(r'\[(.*?)\]', line)[1]) * 1000000)
                # ros_states.append({'timestamp': timestamp, 'identity': identity, 'state': 'icebreak'})
                states_df.loc[len(states_df)] = [timestamp, identity, 'icebreak']


            if "Stage Update: set to " in line:
                # extract the state
                state = line.split("Stage Update: set to ")[1].strip()
                timestamp = int(float(re.findall(r'\[(.*?)\]', line)[1]) * 1000000)
                # ros_states.append({'timestamp': timestamp, 'identity': identity, 'state': sb_states[state]})
                states_df.loc[len(states_df)] = [timestamp, identity, sb_states[state]]


            if "node terminated." in line:
                state = 'end'
                timestamp = int(float(re.findall(r'\[(.*?)\]', line)[1]) * 1000000)
                # ros_states.append({'timestamp': timestamp, 'identity': identity, 'state': state})
                states_df.loc[len(states_df)] = [timestamp, identity, 'end']

            # process user markers: will be like "USER_MARKER [id]: message" 
            if "MARKER" in line:
                # extract the id and message
                timestamp = int(float(re.findall(r'\[(.*?)\]', line)[1]) * 1000000)
                marker_id = int(re.findall(r'\[(.*?)\]', line)[3])
                message = line.split(": ", 2)[2].strip()
                markers.loc[len(markers)] = [timestamp, identity, marker_id, message]

            if "Execution completed!" in line:
                # add to exec_completion. format is {drone_id: timestamp}
                timestamp = int(float(re.findall(r'\[(.*?)\]', line)[1]) * 1000000)
                drone_id = int(re.findall(r'\d+', line.split("Execution completed!")[0])[2])
                exec_completion[drone_id] = timestamp


# now calculate the time gap for each drones, using ros_refrence_ts
timegaps = dict()
for i in range(droneno):
    timegaps[i] = ros_timegaps[i]
    print(f"i: {i}, Time gap: {timegaps[i] - ground_init_time}")

# read and save ulog_msgs as dataframe: .timestamp, id, .message
df_px4_msgs = pd.DataFrame(columns=['timestamp', 'identity', 'message'])
for i, msgs in ulog_msgs.items():
    for msg in msgs:
        df_px4_msgs.loc[len(df_px4_msgs)] = [msg.timestamp + timegaps[i], i, msg.message]
# sort by timestamp
df_px4_msgs.sort_values(by='timestamp', inplace=True)
df_px4_msgs.describe()

df_px4_msgs.to_csv(save_pref_path+'px4_messages.csv', index=False)

ros_states = sorted(ros_states, key=lambda x: x['timestamp'])

dtypes = {
    'timestamp': 'int64',
    'identity': 'object',
    'state': 'object'   
}
# states_df = pd.DataFrame.from_dict(ros_states, orient='columns', dtype=dtypes)
# set timestamp to type timestamp
states_df['timestamp'] = pd.to_datetime(states_df['timestamp'], unit='us')
# remove duplicates of states_df. if the identity and state is same, keep the fastest (smallest timestamp) one
states_df = states_df.sort_values(by='timestamp').drop_duplicates(subset=['identity', 'state'], keep='first')

states_df.rename(columns={'timestamp': 'Start', 'identity': 'identity', 'state': 'state'}, inplace=True)
# sort by 'Task' and 'Start'
states_df.sort_values(by=['identity', 'Start'], inplace=True)
# set 'Finish' as the start time of next state
states_df['Finish'] = states_df.groupby('identity')['Start'].shift(-1)
# fill NaT with maximum timestamp of the entire dataframe
max_timestamp = states_df['Start'].max()
states_df['Finish'] = states_df['Finish'].fillna(max_timestamp)

first_timestamp = states_df['Start'].min()
# let's change start and finish to relative time from the first timestamp
states_df['Start'] = (states_df['Start'] - first_timestamp).dt.total_seconds()
states_df['Finish'] = (states_df['Finish'] - first_timestamp).dt.total_seconds()

# create markers column 'relative_time' as relative time from ground_init_time
markers['timestamp'] = pd.to_datetime(markers['timestamp'], unit='us')
markers['relative_time'] = (markers['timestamp'] - first_timestamp).dt.total_seconds()
# reorder columns: timestamp, relative_time, identity, marker, message
markers = markers[['timestamp', 'relative_time', 'identity', 'marker', 'message']]
# sortby timestamp
markers.sort_values(by='timestamp', inplace=True)



def plot_discrepancies(savefig=True, ma_window=20):
    # ### get latest .csv and copy from sb_log/
    sb_logpath = os.path.expanduser(project_path + 'sb_log/')
    # find latest file
    sb_csvs = sorted([os.path.join(sb_logpath, f) for f in os.listdir(sb_logpath) if f.endswith('.csv')], key=os.path.getctime, reverse=True)
    sb_jsons = sorted([os.path.join(sb_logpath, f) for f in os.listdir(sb_logpath) if f.endswith('.json')], key=os.path.getctime, reverse=True)

    if sb_csvs:
        latest_sb_csv = sb_csvs[0]
        print(f"Latest SB log file: {latest_sb_csv}")
        shutil.copy(latest_sb_csv, save_pref_path)
        print(f"Copied {latest_sb_csv} to {save_pref_path}")
    else:
        print("No SB log files found.")

    # copy sb_jsons into save_pref_path
    if sb_jsons:
        latest_sb_json = sb_jsons[0]
        print(f"Latest json file: {latest_sb_json}")
        shutil.copy(latest_sb_json, save_pref_path)
        print(f"Copied {latest_sb_json} to {save_pref_path}")
    else:
        print("No SB json files found.")

    fig_discs = {}
    if sb_csvs:
        logs = sb_csvs[0]
        logs = pd.read_csv(logs)
        discrepancies = logs[logs['log_type'] == 'Discrepancy'].copy()
        discrepancies['ts_sent'] = pd.to_datetime(discrepancies['ts_sent'], unit='us')
        discrepancies['ts_sent'] = (discrepancies['ts_sent'] - first_timestamp).dt.total_seconds()
        dfs = {drone_id: discrepancies[discrepancies['drone_id'] == drone_id].copy() for drone_id in range(droneno)}
        fig_discs['N'] = go.Figure()
        fig_discs['E'] = go.Figure()
        fig_discs['D'] = go.Figure()

        for drone_id, df in dfs.items():
            df['val1_ma'] = df['val1'].rolling(window=ma_window, min_periods=1).mean()
            df['val2_ma'] = df['val2'].rolling(window=ma_window, min_periods=1).mean()
            df['val3_ma'] = df['val3'].rolling(window=ma_window, min_periods=1).mean()
            fig_discs['N'].add_trace(go.Scatter(x=df['ts_sent'], y=df['val1_ma'], mode='lines', name=f"Drone {drone_id}",
                                                line=dict(width=2, dash=f'{linetypes[drone_id % len(linetypes)]}', color=f'{colors[drone_id % len(colors)]}')))
            fig_discs['E'].add_trace(go.Scatter(x=df['ts_sent'], y=df['val2_ma'], mode='lines', name=f"Drone {drone_id}",
                                                line=dict(width=2, dash=f'{linetypes[drone_id % len(linetypes)]}', color=f'{colors[drone_id % len(colors)]}')))
            fig_discs['D'].add_trace(go.Scatter(x=df['ts_sent'], y=df['val3_ma'], mode='lines', name=f"Drone {drone_id}",
                                                line=dict(width=2, dash=f'{linetypes[drone_id % len(linetypes)]}', color=f'{colors[drone_id % len(colors)]}')))

        for direct, fig_disc in fig_discs.items():
            fig_disc.update_layout(
                title=f'Discrepancy {direct} over Time (MA {ma_window})',
                xaxis_title='Time (s)',
                yaxis_title='Discrepancy (m)',
                # template='simple_white',
                width=1800,
                height=600,
                template='simple_white',
                margin=dict(l=20, r=20, t=60, b=20),
                font_family='Fira Code',
                font_color='black',
                font_size=14,
                font_weight=500,
            )
            if savefig:
                fig_disc.write_image(save_pref_path+f'Discrepancy_{direct}.pdf', scale=2, width=1800, height=600)

if rq3: plot_discrepancies(savefig=True, ma_window=20)

data_vlp = []
data_ts = []
data_vs = []


for i, data in enumerate(ulog_data):
    # local position
    df = pd.DataFrame(data['vehicle_local_position'])[['timestamp', 'x','y','z']]
    df['timestamp'] = df['timestamp'] + timegaps[i]
    df['timestamp'] = pd.to_datetime(df['timestamp'], unit='us')
    df['timestamp'] = (df['timestamp'] - first_timestamp).dt.total_seconds()
    data_vlp.append(df)

    # trajectory setpoint
    df2 = pd.DataFrame(data['trajectory_setpoint'])[['timestamp', 'position[0]','position[1]','position[2]']]
    df2.columns = ['timestamp', 'North', 'East', 'Down']
    df2['timestamp'] = df2['timestamp'] + timegaps[i]
    df2['timestamp'] = pd.to_datetime(df2['timestamp'], unit='us')
    df2['timestamp'] = (df2['timestamp'] - first_timestamp).dt.total_seconds()
    data_ts.append(df2)
    
    # vehicle status
    df5 = pd.DataFrame(data['vehicle_status'])
    df5['timestamp'] = df5['timestamp'] + timegaps[i]
    df5['timestamp'] = pd.to_datetime(df5['timestamp'], unit='us')
    df5['timestamp'] = (df5['timestamp'] - first_timestamp).dt.total_seconds()
    data_vs.append(df5)


# get arm-disarm, nav_state, failsafe state change timestamps
data_vs_processed = []
for drone_id in range(0, droneno):
    df = pd.DataFrame(columns = ['timestamp', 'arm_state_change', 'nav_state_change', 'failsafe_state_change'])
    ref = data_vs[drone_id][['timestamp', 'arming_state', 'nav_state', 'failsafe']]
    df['timestamp'] = ref['timestamp'].shift(1)
    df['arm_state_change'] = (ref['arming_state'] != ref['arming_state'].shift(1))
    df['nav_state_change'] = (ref['nav_state'] != ref['nav_state'].shift(1))
    df['failsafe_state_change'] = (ref['failsafe'] != ref['failsafe'].shift(1))
    # refld = data_ld[drone_id][['timestamp', 'landed']]
    # leave only the rows with at least one state change
    df = df[df[['arm_state_change', 'nav_state_change', 'failsafe_state_change']].any(axis=1)]
    # refld2 = refld[refld['landed'] != refld['landed'].shift(1)]
    # merge df and refld on timestamp
    # df = pd.merge(df, refld2, on='timestamp', how='outer')
    # now find ref values and add as columns
    df['arming_state'] = ref['arming_state'].shift(-1)
    df['nav_state'] = ref['nav_state'].shift(-1)
    df['failsafe'] = ref['failsafe'].shift(-1)
    # df['landed'] = refld['landed'].shift(-1)
    data_vs_processed.append(df)

# let's change this to ['timestamp', 'what', 'from', 'to']
# we'll also process vehicle_land_detected data
# data_vs_processed = []
for drone_id in range(0, droneno):
    df = pd.DataFrame(columns=['timestamp', 'what', 'from', 'to'])
    # data_vs[drone_id]['timestamp'] = data_vs[drone_id]['timestamp'] - timegaps[drone_id]
    ref = data_vs[drone_id][['timestamp', 'arming_state', 'nav_state', 'failsafe']]

    arm_state_change = ref[ref['arming_state'] != ref['arming_state'].shift(1)]
    arm_state_change = arm_state_change[['timestamp', 'arming_state']]
    arm_state_change['what'] = 'arming_state'
    arm_state_change['from'] = arm_state_change['arming_state'].shift(1)
    arm_state_change['to'] = arm_state_change['arming_state']
    arm_state_change = arm_state_change.drop(columns=['arming_state'])

# get timestamps of global first setpoint (last first setpoint)
first_setpoint_complete = 0
for drone_id in range(0, droneno):
    first_setpoint_time = data_ts[drone_id]['timestamp'].iloc[0]  # convert to seconds
    if first_setpoint_complete == 0:
        first_setpoint_complete = first_setpoint_time
    else:
        first_setpoint_complete = min(first_setpoint_complete, first_setpoint_time)

# get time range of flight ()
# get time range of land (nav_state == 18.0)


fig_xy = go.Figure()
fig_xy_sub = make_subplots(rows=DRONES_PER_ROW, cols=DRONES_PER_ROW, subplot_titles=[f'Drone {i}' for i in range(droneno)])

for drone_id in range(0,droneno):
    drone_data = data_vlp[drone_id]
    exec_ends = states_df[(states_df['state']=='execution') & (states_df['identity']==('drone_'+str(drone_id)))]['Finish'].item()
    # setpoints = data_ts[drone_id]
    setpoints = data_ts[drone_id][data_ts[drone_id]['timestamp'] < exec_ends]
    # get setpoints only before the landing stage
    # setpoints_filtered = data_ts[drone_id][]

    #     # airview, delivery
    # north=$((n / x_r * (5)))
    # east=$((n % x_r * (5)))
    # pos = [(drone_id % 5) * 5, (drone_id // 3) * 5]

    traj_x = drone_data['x'].array      + init_pos[drone_id][0]
    traj_y = drone_data['y'].array      + init_pos[drone_id][1]
    setp_x = setpoints['North'].array   + init_pos[drone_id][0]
    setp_y = setpoints['East'].array    + init_pos[drone_id][1]

    fig_xy.add_trace(go.Scatter(x=traj_y, y=traj_x, 
                             mode='lines', legendgroup=f'Drone {drone_id}',
                             legendgrouptitle_text=f'Drone {drone_id}', name='Trajectory',
                             legendgrouptitle_font_size=14,
                             line=dict(width=4, dash=f'{linetypes[drone_id % len(linetypes)]}', color=f'{colors[drone_id % len(colors)]}'),
                             ))
    fig_xy_sub.add_trace(go.Scatter(x=traj_y, y=traj_x, 
                   mode='lines', legendgroup=f'Drone {drone_id}',
                     legendgrouptitle_text=f'Drone {drone_id}', name='Trajectory',
                     legendgrouptitle_font_size=14,
                   line=dict(width=3, dash='solid', color=f'{colors[drone_id % len(colors)]}'),
                   ),
        row=(drone_id // DRONES_PER_ROW) + 1, col=(drone_id % DRONES_PER_ROW) + 1
    )
    if plot_markers:
        fig_xy.add_trace(
            go.Scatter(x=setp_y, y=setp_x, mode='markers', legendgroup=f'Drone {drone_id}', name='Setpoint',
                        marker=dict(size=5, color=f'{colors[drone_id % len(colors)]}', line_color='black', line_width=1, symbol=drone_id)))
        fig_xy_sub.add_trace(
            go.Scatter(x=setp_y, y=setp_x, mode='markers', legendgroup=f'Drone {drone_id}', name='Setpoint',
                    marker_symbol=0,
                        marker=dict(size=2, color='black')),
            row=(drone_id // DRONES_PER_ROW) + 1, col=(drone_id % DRONES_PER_ROW) + 1
        )

# # ############################ for adaptiveswarm ############################
if execinfo.endswith('adaptive'):
    fig_xy.add_shape(type="rect",line=dict(color="RoyalBlue"),
        y1=130, x1=-100, y0=-70, x0=-70)
    fig_xy.add_shape(type="rect",line=dict(color="RoyalBlue"),
        y1=-40, x1=-70, y0=-70, x0=50)
    fig_xy.add_shape(type="rect",line=dict(color="RoyalBlue"),
        y1=30, x1=50, y0=-70, x0=80)
    fig_xy.add_shape(type="rect",line=dict(color="RoyalBlue"),
        y1=-170, x1=-300, y0=-320, x0=-280)
    fig_xy.add_shape(type="rect",line=dict(color="RoyalBlue"),
        y1=-170, x1=-280, y0=-190, x0=-180)
    fig_xy.add_shape(type="rect",line=dict(color="RoyalBlue"),
        y1=130, x1=-350, y0=-370, x0=-347)
    fig_xy.add_shape(type="rect",line=dict(color="RoyalBlue"),
        y1=130, x1=147, y0=-370, x0=150)
    fig_xy.add_shape(type="rect",line=dict(color="RoyalBlue"),
        y1=-367, x1=-347, y0=-370, x0=147)
    fig_xy.add_shape(type="rect",line=dict(color="RoyalBlue"),
        y1=130, x1=-347, y0=127, x0=147)

    # xlim and ylim [-50, 350] for fig_xy
    fig_xy.update_xaxes(range=[-400, 150])
    fig_xy.update_yaxes(range=[-400, 150])
# # ###########################################################################

fig_xy.update_yaxes(scaleanchor="x", scaleratio=1)
fig_xy.update_xaxes(scaleanchor="y", scaleratio=1)
fig_xy.update_layout(
    xaxis_title='East (m)',
    yaxis_title='North (m)',
    width=800,
    height=800,
    template='simple_white',
    margin=dict(l=20, r=20, t=20, b=20),
    font_family='Fira Code',
    font_color='black',
    font_size=14,
    font_weight=500,
    showlegend=False
)
# fig_xy.update_legends(font=dict(size=10), title_font_size=10)
    
fig_xy_sub.update_yaxes(scaleanchor="x", scaleratio=1)
fig_xy_sub.update_xaxes(scaleanchor="y", scaleratio=1)
fig_xy_sub.update_layout(
    # xaxis_title='East (m)',
    # yaxis_title='North (m)',
    width=1000,
    height=800,
    template='simple_white',
    margin=dict(l=20, r=20, t=20, b=20),
    font_family='Fira Code',
    font_color='black',
    font_size=14,
    font_weight=500,
)
fig_xy_sub.update_legends(font=dict(size=14), title_font_size=14)
# fig_xy_sub.show()
# fig_xy_sub.write_image(save_pref_path+'drone_trajectories_subplots.pdf', scale=2, width=1000, height=800)

# pos-x, y, z to time graph
fig_pt = make_subplots(rows=3, cols=1, subplot_titles=['North', 'East', 'Down'])

# fig3 = go.Figure() # North
# fig3 = go.Figure() # East
# fig3 = go.Figure() # Down



for drone_id in range(0,droneno):
    drone_data = data_vlp[drone_id]
    # setpoints = data_ts[drone_id]
    exec_ends = states_df[(states_df['state']=='execution') & (states_df['identity']==('drone_'+str(drone_id)))]['Finish'].item()
    setpoints = data_ts[drone_id][data_ts[drone_id]['timestamp'] < exec_ends]
    pos_x = drone_data['x'].array
    pos_y = drone_data['y'].array
    pos_z = drone_data['z'].array
    setp_x = setpoints['North'].array
    setp_y = setpoints['East'].array
    setp_z = setpoints['Down'].array

    fig_pt.add_trace(go.Scatter(x=drone_data['timestamp'], y=pos_x, legendgroup=f'Drone {drone_id}',
                                legendgrouptitle_text=f'Drone {drone_id}',
                                legendgrouptitle_font_size=12,
                             mode='lines', name=f'Trajectory',
                             line=dict(width=1, dash=f'{linetypes[drone_id % len(linetypes)]}', color=f'{colors[drone_id % len(colors)]}')), row=1, col=1)
    fig_pt.add_trace(go.Scatter(x=drone_data['timestamp'], y=pos_y, legendgroup=f'Drone {drone_id}',
                             mode='lines', name=f'Trajectory', showlegend=False,
                             line=dict(width=1, dash=f'{linetypes[drone_id % len(linetypes)]}', color=f'{colors[drone_id % len(colors)]}')), row=2, col=1)
    fig_pt.add_trace(go.Scatter(x=drone_data['timestamp'], y=pos_z, legendgroup=f'Drone {drone_id}',
                             mode='lines', name=f'Trajectory', showlegend=False,
                             line=dict(width=1, dash=f'{linetypes[drone_id % len(linetypes)]}', color=f'{colors[drone_id % len(colors)]}')), row=3, col=1)
    if plot_markers:
        fig_pt.add_trace(go.Scatter(x=setpoints['timestamp'], y=setp_x, legendgroup=f'Drone {drone_id}',
                                mode='markers', name=f'Setpoint',
                                marker=dict(size=5, symbol=drone_id, color=f'{colors[drone_id % len(colors)]}', 
                                    # line_color='black', line_width=1
                                    ), 
                                ),row=1, col=1)
        fig_pt.add_trace(go.Scatter(x=setpoints['timestamp'], y=setp_y, legendgroup=f'Drone {drone_id}',
                                mode='markers', name=f'Setpoint', showlegend=False,
                                marker=dict(size=5, symbol=drone_id, color=f'{colors[drone_id % len(colors)]}', 
                                    # line_color='black', line_width=1
                                    ), 
                                ),row=2, col=1)
        fig_pt.add_trace(go.Scatter(x=setpoints['timestamp'], y=setp_z, legendgroup=f'Drone {drone_id}',
                                mode='markers', name=f'Setpoint', showlegend=False,
                                marker=dict(size=5, symbol=drone_id, color=f'{colors[drone_id % len(colors)]}', 
                                    # line_color='black', line_width=1
                                    ), 
                                ),row=3, col=1)
    # add vertical line for state changes
    for index, row in data_vs_processed[drone_id].iterrows():
        timest = row['timestamp']
        if index == 0:
            continue # this should be only at the first row
        if row['arm_state_change']:
            # fig3.add_shape(type='line', x0=timest, y0=-10, x1=timest, y1=10,
            #                 line=dict(color=f'{colors[drone_id % len(colors)]}', width=1, dash='longdash'))
            fig_pt.add_vline(x=timest, line_width=1, line_dash='longdash', line_color=f'{colors[drone_id % len(colors)]}', row='all', col=1)
        if row['nav_state_change']:
            fig_pt.add_vline(x=timest, line_width=1, line_dash='dot', line_color=f'{colors[drone_id % len(colors)]}', row='all', col=1)
        if row['failsafe_state_change']:
            fig_pt.add_vline(x=timest, line_width=1, line_dash='dashdot', line_color=f'{colors[drone_id % len(colors)]}', row='all', col=1)
    # instead, let's use add_vrect for each same state changes

# draw vertical lines for stage changes of ground
# reset index of states_df
states_df.reset_index(drop=True, inplace=True)
indcount = 0
for index, row in states_df[states_df['identity'] == 'ground'].iterrows():
    # fig3.add_vline(x=row['Start'], line_width=1, line_dash='solid', line_color='black',
    #             #    annotation_text='stage change', annotation_position='top left',
    #                row='all', col=1)
    fig_pt.add_vrect(x0=row['Start'], x1=row['Finish'], 
                   fillcolor=colors[indcount % len(colors)], 
                   opacity=0.1, 
                   line_width=1, #    annotation_text=f'Stage: {row["state"]}',
                   label=dict(
                    #    text=f'{row["state"]}',
                       text='' if (row['state'] == 'icebreak') else f'{row["state"]}',
                       textposition='bottom left',
                    #    textposition='bottom right' if (row['state']=='icebreak') else 'bottom left',
                       textangle=-90,
                       font=dict(size=10, color='black', weight='normal'),
                    #    opacity=0 if (row['state'] == 'icebreak') else 1,
                       padding=10
                   ),
                   layer='below')
    indcount += 1

fig_pt.update_layout(
    # xaxis_title='Time (s)',
    # yaxis_title='Position (m)',
    width=1200,
    height=800,
    template='simple_white',
    margin=dict(l=20, r=20, t=20, b=20),
    font_family='Fira Code',
    font_color='black',
    font_size=14,
    font_weight=500,
)
fig_pt.update_xaxes(title_text = 'Time (s)', range=[-(states_df['Finish'].max())*0.02, states_df['Finish'].max()*1.02])
fig_pt.update_yaxes(title_text = 'Position (m)')
fig_pt.update_legends(font=dict(size=12))
# fig_pt.write_image(save_pref_path+'drone_position-time.pdf', scale=2, width=1200, height=800)
# fig_pt.write_image(save_pref_path+'drone_position-time.png', scale=2, width=1200, height=800)
# fig_pt.show()

# compare flight distance of each drone: we'll show them as cumulative distance traveled chart (time-distance)
fig_dist = go.Figure()
fig_bar = go.Figure()
patterns = ['.', '/', '\\', 'x', '-', '|', '+']
cum_dist = {}

for drone_id in range(0, droneno):
    # first compute the cumulative distance traveled
    # we will use the x and y coordinates of the drone (ignore z)
    drone_data = data_vlp[drone_id]
    dx = np.diff(drone_data['x'].array)
    dy = np.diff(drone_data['y'].array)
    dp = np.sqrt(dx**2 + dy**2)
    cumulative_distance = np.cumsum(dp)
    # Compute the total distance traveled by the drone
    total_distance = cumulative_distance[-1] if len(cumulative_distance) > 0 else 0
    # Store the cumulative distance for the drone
    cum_dist[drone_id] = float(total_distance)
    # Add the cumulative distance as a new line in the figure
    fig_dist.add_trace(go.Scatter(
        x=drone_data['timestamp'][1:],  # skip the first timestamp as it has no previous point
        y=cumulative_distance,
        mode='lines',
        name=f'Drone {drone_id} (Total: {total_distance:.2f} m)',
        line=dict(width=2, color=f'{colors[drone_id % len(colors)]}')
    ))
    fig_bar.add_trace(go.Bar(
        y=[total_distance],
        x=[f'Drone {drone_id}'],
        # name=f'distance',
        marker_color=f'{colors[drone_id % len(colors)]}',
        text=f'{total_distance:.2f} m',
        textposition='outside',
        marker_pattern_shape= patterns[drone_id % len(patterns)],
        # orientation='h'
        showlegend=False,        
    ))
    
fig_dist.update_layout(
    xaxis_title='Time',
    yaxis_title='Distance (m)',
    width=1200,
    height=600,
    template='simple_white',
    margin=dict(l=20, r=20, t=20, b=20),
    font_family='Fira Code',
    font_color='black',
    font_size=14,
    font_weight=500,
    # showlegends=False,
)
# fig_dist.update_xaxes(tickangle=-45)
# fig_dist.update_yaxes(tickformat='.2f')

# fig_dist.write_image(save_pref_path+'drone_cumsum_distance.pdf', scale=2, width=1200, height=600)
# fig_dist.show()


fig_bar.update_layout(
    xaxis_title='Drone',
    yaxis_title='Total Distance (m)',
    width=800,
    height=800,
    template='simple_white',
    # margin=dict(l=20, r=20, t=20, b=20),
    font_family='Fira Code',
    font_color='black',
    font_size=14,
    font_weight=500,
    barmode='group',
)
# fig_bar.update_yaxes(tickformat='.2f')
# fig_bar.update_xaxes(tickangle=-45)

fig_bar.update_traces(
    marker=dict(
        # color='gray',
        line_color='black',
        pattern_fillmode='overlay',
        line_width=2,
        pattern_fgcolor='black',
        pattern_fgopacity=1,
        pattern_size=8,
        # pattern_shape= ['/', '\\', 'x', '-', '|', '+', '.']
    ),
    # pattern=dict(
    #     fgcolor='black',
    # ),
    # marker_line_color='black', marker_line_width=2, marker_color='black'
    )

# fig_bar.write_image(save_pref_path+'drone_total_distance.pdf', scale=2, width=1200, height=600)
# fig_bar.show()

# calculate ground execution stage time consumption (execution stage is from 'mission execution' to 'landing (completion)')
ground_execution_time = states_df[(states_df['identity'] == 'ground') & (states_df['state'] == 'execution')]
exec_start = ground_execution_time['Start'].values[0] if not ground_execution_time.empty else 0
if not ground_execution_time.empty:
    ground_execution_time = ground_execution_time['Finish'].values[0] - ground_execution_time['Start'].values[0]
else:
    ground_execution_time = 0

# calculate utilization metrics: utilization rate / idle time of each drone, standard deviation of {utilization time(execution completion time - execution start time), flight distance}
        # Utilization rate              & Average drone utilization rate during the execution stage     & $\%$ \\
        # Idle time                     & Average idle time (between delivery completion and the end of execution stage)& $s$ \\
        # Workload balance (distance)   & The standard deviation of flight distance per drone     & $m$ \\
        # Workload balance (time)       & The standard deviation of utilization time per drone     & $s$ \\
utilization_rate = {}
idle_time = {}
workload_time = {}
# workload_distance is the same as cumulative distance
for drone_id in range(0, droneno):
    # work_time = exec_completion[drone_id] - exec_start if drone_id in exec_completion else 0
    work_time = (exec_completion[drone_id] - ground_init_time)/1000000 if drone_id in exec_completion else 0
    if work_time < 0:
        work_time = 0
    workload_time[drone_id] = min(work_time, ground_execution_time)
    # utilization rate = work_time / ground_execution_time
    if ground_execution_time > 0:
        idle_time[drone_id] = ground_execution_time - workload_time[drone_id]
        utilization_rate[drone_id] = (workload_time[drone_id] / ground_execution_time) * 100
    else:
        utilization_rate[drone_id] = 0
        idle_time[drone_id] = 0


print("---creating outputs---")

json_data = {
        'config': str(config_path),
        # cost efficiency
        'flight_distance': sum(cum_dist.values()),
        'mission_duration': ground_execution_time,
        # resource utilization
        'utilization_rate': utilization_rate,
        'idle_time': idle_time,
        'workload_time': workload_time,
        'workload_distance': cum_dist,
}

# calculate delivery time.
if strategy:
    delivery_time = pd.DataFrame(columns=['package_id', 'drone_id', 'delivery_time'])
    # read markers where identity is ground
    for index, row in markers[markers['identity'] == 'ground'].iterrows():
        # read marker id 2
        if row['marker'] == 2:
            # parse "Drone XX completed delivery of package YY." -> XX: drone_id, YY: package_id
            message = row['message']
            # if "completed delivery of package" in message:
            parts = message.split("completed delivery of package")
            drone_id = int(parts[0].split("Drone ")[1].strip())
            package_id = int(parts[1].strip()[:-1])
            delivery_time.loc[len(delivery_time)] = [package_id, drone_id, row['relative_time'] - exec_start]
    json_data['delivery_time'] = delivery_time['delivery_time'].describe().to_dict()
    delivery_time.to_csv(save_pref_path + 'delivery_time.csv', index=False)


# save power consumptions and cumulative distance to json file
with open(save_pref_path + 'results.json', 'w') as f:
    json.dump(json_data, f, indent=4)

markers.to_csv(save_pref_path + 'markers.csv', index=False)

fig_xy.write_image(save_pref_path+'drone_trajectories.pdf', scale=2, width=750, height=750)
fig_xy_sub.write_image(save_pref_path+'drone_trajectories_subplots.pdf', scale=2, width=1000, height=800)
fig_dist.write_image(save_pref_path+'drone_cumsum_distance.pdf', scale=2, width=1200, height=600)
fig_bar.write_image(save_pref_path+'drone_total_distance.pdf', scale=2, width=600, height=600)
if not execinfo.endswith('socratic'):
    # for rq1_socratic (SocraticSwarm), the p-t plot will require too much resources and is not necessary to analyze the results, so we will skip it.
    fig_pt.write_image(save_pref_path+'drone_position-time.pdf', scale=2, width=1200, height=800)

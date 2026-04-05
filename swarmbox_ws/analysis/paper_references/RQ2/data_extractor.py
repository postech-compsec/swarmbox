import pandas as pd

# read ulog files at physical_raw and sitl_raw folders
import pyulog
import os

physical_logs = []
sitl_logs = []

for root, dirs, files in os.walk('data/physical_raw'):
    for file in files:
        if file.endswith('.ulg'):
            physical_logs.append(os.path.join(root, file))

for root, dirs, files in os.walk('data/sitl_raw'):
    for file in files:
        if file.endswith('.ulg'):
            sitl_logs.append(os.path.join(root, file))

# sort logs
physical_logs.sort()
sitl_logs.sort()

# print log list
print("Physical Logs:")
print(physical_logs)
print("SITL Logs:")
print(sitl_logs)

# 'vehicle_local_position' to csvs


sitl_ulgs = [pyulog.ULog(log) for log in sitl_logs]
physical_ulgs = [pyulog.ULog(log) for log in physical_logs]

sitl_local_pos = [ulg.get_dataset('vehicle_local_position').data for ulg in sitl_ulgs]
physical_local_pos = [ulg.get_dataset('vehicle_local_position').data for ulg in physical_ulgs]

sitl_dfs = []
phys_dfs = []

for i in range(len(sitl_local_pos)):
    sitl_df = pd.DataFrame(sitl_local_pos[i])
    # get only x, y, z, timestamp
    sitl_df = sitl_df[['x', 'y', 'z', 'timestamp']]
    sitl_dfs.append(sitl_df)
    phys_df = pd.DataFrame(physical_local_pos[i])
    # get only x, y, z, timestamp
    phys_df = phys_df[['x', 'y', 'z', 'timestamp']]
    phys_dfs.append(phys_df)

# add relative position values to x, y columns. they are positioned as:
initial_positions = [
    [  0.0,   0.0],
    [ -3.0,  -3.0],
    [ -3.0,   3.0],
    [ -6.0,  -6.0]
]

for i in range(len(sitl_dfs)):
    sitl_dfs[i]['x'] += initial_positions[i][0]
    sitl_dfs[i]['y'] += initial_positions[i][1]
    phys_dfs[i]['x'] += initial_positions[i][0]
    phys_dfs[i]['y'] += initial_positions[i][1]

# now the dataframes have absolute positions
# 
# save to csvs
for i in range(len(sitl_dfs)):
    sitl_dfs[i].to_csv(f'sitl_local_position_{i}.csv', index=False)
    phys_dfs[i].to_csv(f'physical_local_position_{i}.csv', index=False)
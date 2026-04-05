import os
import glob
import json
import pandas as pd

directory = './analysis/data/rq3_network_fault'

folders = sorted(glob.glob(os.path.join(directory, "rq3_faulty_*")))
rows = []

for folder in folders:
    param_path = glob.glob(os.path.join(folder, "params_*.json"))
    with open(param_path[0], 'r') as f:
        params = json.load(f)

    ground_path = glob.glob(os.path.join(folder, "ground*.csv"))
    ground = pd.read_csv(ground_path[0])

    exec = ground[ground["log_type"] == "EXEC_STAGE"]
    exec_time = exec.iloc[0]['ts_sent']
    
    loss = None
    latency = None

    for i in range(exec.index[0] + 1, len(ground), 1):
        row = ground.iloc[i]
        if row["log_type"] == "Loss" and loss is None:
            loss = row
        elif row["log_type"] == "Latency" and latency is None:
            latency = row
        if loss is not None and latency is not None:
            break
    
    loss_row = {
        'Target': params.get('target', ''),
        'Delay': params.get('delay', ''),
        'Loss': params.get('loss', ''),
        'type': 'Loss',
        'Time': loss['ts_sent'] - exec_time if loss is not None else '',
        'val1': loss.get('val1', '') if loss is not None else '',
        'val2': loss.get('val2', '') if loss is not None else '',
        'val3': loss.get('val3', '') if loss is not None else ''
    }
    rows.append(loss_row)
    
    latency_row = {
        'Target': params.get('target', ''),
        'Delay': params.get('delay', ''),
        'Loss': params.get('loss', ''),
        'type': 'Latency',
        'Time': latency['ts_sent'] - exec_time if latency is not None else '',
        'val1': latency.get('val1', '') if latency is not None else '',
        'val2': latency.get('val2', '') if latency is not None else '',
        'val3': latency.get('val3', '') if latency is not None else ''
    }
    rows.append(latency_row)

df = pd.DataFrame(rows)
    
column_order = ['Target', 'Delay', 'Loss', 'type', 'Time', 'val1', 'val2', 'val3']
df = df[column_order]
df.to_csv("rq3_network_df.csv")
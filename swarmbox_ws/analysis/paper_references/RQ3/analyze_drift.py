import os
import glob
import json
import pandas as pd

directory = './analysis/data'

folders = sorted(glob.glob(os.path.join(directory, "rq3_faulty_*")))
rows = []

for folder in folders:
    param_path = glob.glob(os.path.join(folder, "params_*.json"))
    with open(param_path[0], 'r') as f:
        params = json.load(f)

    ground_path = glob.glob(os.path.join(folder, "ground*.csv"))
    ground = pd.read_csv(ground_path[0])

    px4_ = glob.glob(os.path.join(folder, "px4_messages.csv"))
    px4_df = pd.read_csv(px4_[0]) if px4_ else pd.DataFrame()
    px4_first_log_t = px4_df['timestamp'].values[0] if not px4_df.empty else None

    estimator_resets = glob.glob(os.path.join(folder, "fordrift_estimator_resets.csv"))
    estimator_resets_df = pd.read_csv(estimator_resets[0]) if estimator_resets else pd.DataFrame()

    
    sys_detected = ground[(ground['log_type']=='Discrepancy_N') & (ground['val1'] == params['target'])].iloc[0]
    sb_row = {
        'Target': params.get('target', ''),
        'Drift': params.get('drift_value', ''),
        'Inject': params.get('injection_time', ''),
        'type': 'sys',
        'Time': (sys_detected['ts_sent'] - px4_first_log_t - 30*1000*1000) if sys_detected is not None else '',
        'val1': sys_detected.get('val1', '') if sys_detected is not None else '',
        'val2': sys_detected.get('val2', '') if sys_detected is not None else '',
        'val3': sys_detected.get('val3', '') if sys_detected is not None else ''
    }
    rows.append(sb_row)

    baseline_reset_t = estimator_resets_df[(estimator_resets_df['drone_id'] == params['target'])
                                           & (estimator_resets_df['timestamp'] >= 30*1000*1000)]['timestamp']
    baseline_first_reset_t = baseline_reset_t.iloc[0] if not baseline_reset_t.empty else None
    # create line with discrepancy value from log?
    base_detected = ground[(ground['log_type']=='Discrepancy_N') & (ground['val1'] == params['target']) 
                           & (ground['ts_sent'] >= (baseline_first_reset_t + px4_first_log_t))
                           ].iloc[0] if baseline_first_reset_t is not None else None
    base_row = {
        'Target': params.get('target', ''),
        'Drift': params.get('drift_value', ''),
        'Inject': params.get('injection_time', ''),
        'type': 'baseline',
        'Time': (base_detected['ts_sent'] - px4_first_log_t - 30*1000*1000) if base_detected is not None else '',
        'val1': base_detected.get('val1', '') if base_detected is not None else '',
        'val2': base_detected.get('val2', '') if base_detected is not None else '',
        'val3': base_detected.get('val3', '') if base_detected is not None else ''
    }
    rows.append(base_row)

df = pd.DataFrame(rows)
    
column_order = ['Target', 'Drift', 'Inject', 'type', 'Time', 'val1', 'val2', 'val3']
df = df[column_order]
df.to_csv("rq3_drift_df.csv")
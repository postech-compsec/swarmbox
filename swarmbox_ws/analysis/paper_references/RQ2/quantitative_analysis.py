import numpy as np
import pandas as pd
from scipy.spatial import cKDTree

log_dir = 'run/RQ2'

# import from csvs
sitl_dfs = []
phys_dfs = []
for i in range(4):
    sitl_df = pd.read_csv(f'{log_dir}/sitl_local_position_{i}.csv')
    sitl_dfs.append(sitl_df)
    phys_df = pd.read_csv(f'{log_dir}/physical_local_position_{i}.csv')
    phys_dfs.append(phys_df)

# compute RMSE for each log
for i in range(len(sitl_dfs)):
    tree = cKDTree(sitl_dfs[i][['x', 'y']])
    distances, _ = tree.query(phys_dfs[i][['x', 'y']])
    rmse = np.sqrt(np.mean(distances**2))
    print(f'Log {i}: RMSE = {rmse} meters')

# plot if needed
import matplotlib.pyplot as plt
plots = False
if plots:
    # plot x-y for each log
    for i in range(len(sitl_dfs)):
        plt.figure(figsize=(10, 6))
        plt.plot(sitl_dfs[i]['y'], sitl_dfs[i]['x'], label='SITL', alpha=0.7)
        plt.plot(phys_dfs[i]['y'], phys_dfs[i]['x'], label='Physical', alpha=0.7)
        plt.title(f'Log {i} - N-E Position')
        plt.xlabel('N (meters)')
        plt.ylabel('E (meters)')
        plt.legend()
        plt.axis('equal')
        plt.grid()
        plt.show()

    # x-y plot for all physical drones together
    plt.figure(figsize=(10, 6))
    for i in range(len(phys_dfs)):
        plt.plot(phys_dfs[i]['y'], phys_dfs[i]['x'], label=f'Physical Drone {i}', alpha=0.7)
    plt.title('All Physical Drones - N-E Position')
    plt.xlabel('N (meters)')
    plt.ylabel('E (meters)')
    plt.legend()
    plt.axis('equal')
    plt.grid()
    plt.show()

    plt.figure(figsize=(10, 6))
    for i in range(len(sitl_dfs)):
        plt.plot(sitl_dfs[i]['y'], sitl_dfs[i]['x'], label=f'SITL Drone {i}', alpha=0.7)
    plt.title('All SITL Drones - N-E Position')
    plt.xlabel('N (meters)')
    plt.ylabel('E (meters)')
    plt.legend()
    plt.axis('equal')
    plt.grid()
    plt.show()
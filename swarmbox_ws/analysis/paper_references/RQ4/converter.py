import pandas as pd
import os, json

# read all json files
data_dir = "sb_rq4_data" # CONFIG: change this to your data directory!
all_results = []
delivery_data = []
    
for folder in os.listdir(data_dir):
    folder_path = os.path.join(data_dir, folder)
    if os.path.isdir(folder_path):
        results = os.path.join(folder_path, "results.json")
        delivery = os.path.join(folder_path, "delivery_time.csv")
        if os.path.exists(delivery):
            delivery_data.append(delivery)
        if os.path.exists(results):
            with open(results, 'r') as f:
                data = json.load(f)
                all_results.append(data)

data_collections = {
    "power_consumptions": [],
    "utilization_rate": [],
    "idle_time": [],
    "workload_time": [],
    "workload_distance": [],
    "extra": []
}

fields = ["power_consumptions", "utilization_rate", "idle_time", "workload_time", "workload_distance"]
    
# process each result
for data in all_results:
    config_val = data["config"].split("/")[-1]  # extract 'rq4_1.yaml'
    sort_val = int(data["strategy"]["sort"])
    assign_val = int(data["strategy"]["assign"])
    
    base_row = {"sort": sort_val, "assign": assign_val, "config": config_val}
    
    for field in fields:
        row = base_row.copy()
        for i in range(5): # 0-4 indices for drones
            value = float(data[field][str(i)])
            row[f"drone_{i}"] = value
        data_collections[field].append(row)

    # extra data
    extra_row = base_row.copy()
    extra_row.update({
        "flight_distance": data["flight_distance"],
        "mission_duration": data["mission_duration"],
        "delivery_time_mean": data["delivery_time"]["mean"],
        "delivery_time_std": data["delivery_time"]["std"],
        "delivery_time_max": data["delivery_time"]["max"]
    })
    data_collections["extra"].append(extra_row)

# create dataframes
dataframes = {}
index_cols = ["sort", "assign", "config"]

for name, data in data_collections.items():
    df = pd.DataFrame(data)
    df = df.drop_duplicates(subset=index_cols, keep='first')
    df = df.set_index(index_cols)
    dataframes[name] = df


# save df to csv
for name, df in dataframes.items():
    df.to_csv(f"{name}.csv")


# ------- make deliverytime dataframe -------
# the delivery time datas are at: for example, ./sb_rq4_data/rq4_1_s0_a0_20250801_191232/delivery_time.csv
delivery_time_data = []
for data in delivery_data:
    # config_val = data["config"].split("/")[-1]  # extract 'rq4_1.yaml'
    chunks = data.split("/")[1].split("_")
    config_val = "rq4_"+chunks[1]+".yaml"
    sort_val = int(chunks[2].replace("s", ""))
    assign_val = int(chunks[3].replace("a",""))
    
    # delivery_time_path = os.path.join(data_dir, data["config"], "delivery_time.csv")
    if os.path.exists(data):
        delivery_df = pd.read_csv(data)
        delivery_times = delivery_df['delivery_time'].tolist()
        
        row = {
            "sort": sort_val,
            "assign": assign_val,
            "config": config_val,
            "delivery_time": delivery_times
        }
        delivery_time_data.append(row)

# create delivery time dataframe
delivery_time_df = pd.DataFrame(delivery_time_data)
# delivery_time_df = delivery_time_df.drop_duplicates(subset=index_cols, keep='first')
delivery_time_df.to_csv('delivery_time.csv', index=False)
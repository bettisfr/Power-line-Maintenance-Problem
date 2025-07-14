import os
import pandas as pd

experiments_dir = 'output'
plots_dir = 'plots'

# Load all CSVs
df_list = []
for filename in os.listdir(experiments_dir):
    if filename.endswith('.csv'):
        file_path = os.path.join(experiments_dir, filename)
        df = pd.read_csv(file_path)
        df_list.append(df)

# Merge and save master CSV
final_df = pd.concat(df_list, ignore_index=True)
os.makedirs(plots_dir, exist_ok=True)
final_df.to_csv(f'{plots_dir}/all.csv', index=False)

# Reload for filtering
all_df = pd.read_csv(f'{plots_dir}/all.csv')

# Parameter vectors
DRONE_LOAD_VEC = [5, 10]
DRONE_BATTERY_VEC = [2500, 5000]
MAX_WEIGHT_VEC = [1, 5]
ENERGY_PER_DELIVERY_VEC = [0, 30]
EXHAUSTIVE_VEC = [0, 1]
ZIPF_EXPONENT_VEC = [0, 1, 2]
ALGORITHMS_VEC = [0, 8, 9] # 0, 8
# ALGORITHMS_VEC = [0, 2, 3, 6, 8]
# ALGORITHMS_VEC = [0]
MAX_INTERVAL_LEN_VEC = [2, 4]
DELTA_VEC = [1.0, 0.5]

# Iterate over all combinations
for algorithm in ALGORITHMS_VEC:
    for zipf_exponent in ZIPF_EXPONENT_VEC:
        for delta in DELTA_VEC:
            for max_interval_len in MAX_INTERVAL_LEN_VEC:
                for drone_load in DRONE_LOAD_VEC:
                    for drone_battery in DRONE_BATTERY_VEC:
                        for max_weight in MAX_WEIGHT_VEC:
                            for energy_per_delivery in ENERGY_PER_DELIVERY_VEC:
                                for exhaustive in EXHAUSTIVE_VEC:

                                    # Determine solution space
                                    if exhaustive == 1:
                                        STR_SOLUTION_SPACE = "exhaustive"
                                        solution_space = 0 if energy_per_delivery == 0 else 3
                                    else:
                                        STR_SOLUTION_SPACE = "dp"
                                        solution_space = 1 if energy_per_delivery == 0 else 4

                                    # Determine problem string
                                    if max_weight == 1:
                                        STR_PROB = "tmp-u" if energy_per_delivery == 0 else "tmp-ud"
                                    else:
                                        STR_PROB = "tmp-a" if energy_per_delivery == 0 else "tmp-ad"

                                    # Filter the DataFrame
                                    filtered_df = all_df[
                                        (all_df['algorithm'] == algorithm) &
                                        (all_df['max_interval_len'] == max_interval_len) &
                                        (all_df['deliveries_starting_point'] == delta) &
                                        (all_df['drone_load'] == drone_load) &
                                        (all_df['drone_battery'] == drone_battery) &
                                        (all_df['max_weight'] == max_weight) &
                                        (all_df['solution_space'] == solution_space) &
                                        (all_df['energy_per_delivery'] == energy_per_delivery) &
                                        (all_df['exponent'] == zipf_exponent)
                                        ].sort_values(by='num_deliveries')

                                    # Compose file name
                                    # filename = f"{STR_PROB}_{STR_SOLUTION_SPACE}_alg{algorithm}_load{drone_load}_batt{drone_battery}_zipf{zipf_exponent}.csv"
                                    filename = (f"{STR_PROB}"
                                                f"_{STR_SOLUTION_SPACE}"
                                                f"_alg{algorithm}"
                                                f"_load{drone_load}"
                                                f"_batt{drone_battery}"
                                                f"_maxint{max_interval_len}"
                                                f"_delta{delta}"
                                                f"_zipf{zipf_exponent}"
                                                f".csv")
                                    filepath = os.path.join(plots_dir, filename)

                                    # Save if not empty
                                    if not filtered_df.empty:
                                        filtered_df.to_csv(filepath, index=False)
                                        print(f"Saved: {filename} ({len(filtered_df)} rows)")
                                    else:
                                        print(f"Skipped: {filename} (no data)")

# For ratio between alg0 and alg8

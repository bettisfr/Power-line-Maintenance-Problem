import os

import pandas as pd

experiments_dir = "output"
plots_dir = "plots_real"
target_num_deliveries = 79

# Real-instance setup (aligned with run_real_exps.py)
DRONE_LOAD_VEC = [5, 10]
DRONE_BATTERY_VEC = [2500, 5000]
MAX_WEIGHT_VEC = [1, 5]
ENERGY_PER_DELIVERY_VEC = [0, 30]
EXHAUSTIVE_VEC = [0]
ZIPF_EXPONENT_VEC = [0]
ALGORITHMS_VEC = [0, 2, 3, 6]
MAX_INTERVAL_LEN_VEC = [2]
DELTA_VEC = [0.5]


def load_real_instance_df():
    df_list = []
    for filename in os.listdir(experiments_dir):
        if not filename.endswith(".csv"):
            continue
        # Real-instance experiments were generated at ndel79.
        if f"_ndel{target_num_deliveries}_" not in filename:
            continue
        file_path = os.path.join(experiments_dir, filename)
        df_list.append(pd.read_csv(file_path))

    if not df_list:
        return pd.DataFrame()

    final_df = pd.concat(df_list, ignore_index=True)
    final_df = final_df[
        (final_df["instance_type"] == 1) &
        (final_df["num_deliveries"] == target_num_deliveries)
    ]
    return final_df


all_df = load_real_instance_df()
os.makedirs(plots_dir, exist_ok=True)

if all_df.empty:
    print("No real-instance CSVs found for ndel79 in output/.")
    raise SystemExit(0)

all_df.to_csv(f"{plots_dir}/all_real_ndel79.csv", index=False)
print(f"Saved: {plots_dir}/all_real_ndel79.csv ({len(all_df)} rows)")

for algorithm in ALGORITHMS_VEC:
    for zipf_exponent in ZIPF_EXPONENT_VEC:
        for delta in DELTA_VEC:
            for max_interval_len in MAX_INTERVAL_LEN_VEC:
                for drone_load in DRONE_LOAD_VEC:
                    for drone_battery in DRONE_BATTERY_VEC:
                        for max_weight in MAX_WEIGHT_VEC:
                            for energy_per_delivery in ENERGY_PER_DELIVERY_VEC:
                                for exhaustive in EXHAUSTIVE_VEC:
                                    if exhaustive == 1:
                                        str_solution_space = "exhaustive"
                                        solution_space = 0 if energy_per_delivery == 0 else 3
                                    else:
                                        str_solution_space = "dp"
                                        solution_space = 1 if energy_per_delivery == 0 else 4

                                    if max_weight == 1:
                                        str_prob = "tmp-u" if energy_per_delivery == 0 else "tmp-ud"
                                    else:
                                        str_prob = "tmp-a" if energy_per_delivery == 0 else "tmp-ad"

                                    filtered_df = all_df[
                                        (all_df["algorithm"] == algorithm) &
                                        (all_df["max_interval_len"] == max_interval_len) &
                                        (all_df["deliveries_starting_point"] == delta) &
                                        (all_df["drone_load"] == drone_load) &
                                        (all_df["drone_battery"] == drone_battery) &
                                        (all_df["max_weight"] == max_weight) &
                                        (all_df["solution_space"] == solution_space) &
                                        (all_df["energy_per_delivery"] == energy_per_delivery) &
                                        (all_df["exponent"] == zipf_exponent) &
                                        (all_df["instance_type"] == 1) &
                                        (all_df["num_deliveries"] == target_num_deliveries)
                                    ]

                                    filename = (
                                        f"{str_prob}"
                                        f"_{str_solution_space}"
                                        f"_alg{algorithm}"
                                        f"_ndel{target_num_deliveries}"
                                        f"_load{drone_load}"
                                        f"_batt{drone_battery}"
                                        f"_maxint{max_interval_len}"
                                        f"_delta{delta}"
                                        f"_zipf{zipf_exponent}"
                                        f".csv"
                                    )
                                    filepath = os.path.join(plots_dir, filename)

                                    if not filtered_df.empty:
                                        filtered_df.to_csv(filepath, index=False)
                                        print(f"Saved: {filename} ({len(filtered_df)} rows)")
                                    else:
                                        print(f"Skipped: {filename} (no data)")

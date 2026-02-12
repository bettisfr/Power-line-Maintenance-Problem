import os
import subprocess

# Parameters
BUILD_DIR = "cmake-build-release"
BIN_FILE = "dcoss"

os.makedirs("output", exist_ok=True)

# Default parameter values
DEFAULT_LOG = 0
DEFAULT_ITERATIONS = 1
DEFAULT_MAX_LEN_ROAD = 100
DEFAULT_MAX_PROFIT = 10
DEFAULT_HEIGHT = 0.025
DEFAULT_DISTANCE = 0.5
DEFAULT_ENERGY_UNIT_COST = 150
DEFAULT_SAVE = 1
DEFAULT_ERROR = 0.05  # 5% error

########################################################################################################################
# Real-instance experiment setup
INSTANCE_TYPE = 1  # 0 = random/regularly spaced, 1 = real instance
NUM_DELIVERIES = 79
DEFAULT_REGULARLY_SPACED = 1
MAX_INTERVAL_LEN_VEC = [2]
DELIVERIES_STARTING_POINT_VEC = [0.5]
ZIPF_EXPONENT_VEC = [0]
# 0 = OPT (ILP), 2 = DP (knapsack), 3 = COL (coloring), 6 = GMP/E
ALGORITHMS_VEC = [0, 2, 3, 6]
EXHAUSTIVE = 0  # 1 = exhaustive, 0 = DP

# Four problems: (unitary/arbitrary weights) x (no extra cost/extra delivery cost)
MAX_WEIGHT_VEC = [1, 5]
ENERGY_PER_DELIVERY_VEC = [0, 30]

# Two levels for payload/energy budget, as in previous experiments
DRONE_LOAD_VEC = [5, 10]
DRONE_BATTERY_VEC = [2500, 5000]
########################################################################################################################

seed = 0
run_count = 0

for energy_per_delivery in ENERGY_PER_DELIVERY_VEC:
    for max_weight in MAX_WEIGHT_VEC:
        if EXHAUSTIVE == 1:
            str_solution_space = "exhaustive"
            solution_space = 0 if energy_per_delivery == 0 else 3
        else:
            str_solution_space = "dp"
            solution_space = 1 if energy_per_delivery == 0 else 4

        if max_weight == 1:
            str_prob = "tmp-u" if energy_per_delivery == 0 else "tmp-ud"
        else:
            str_prob = "tmp-a" if energy_per_delivery == 0 else "tmp-ad"

        for zipf_exponent in ZIPF_EXPONENT_VEC:
            for drone_battery in DRONE_BATTERY_VEC:
                for drone_load in DRONE_LOAD_VEC:
                    for max_interval_len in MAX_INTERVAL_LEN_VEC:
                        for deliveries_starting_point in DELIVERIES_STARTING_POINT_VEC:
                            for algorithm in ALGORITHMS_VEC:
                                exp_name = (
                                    f"out_{str_prob}"
                                    f"_{str_solution_space}"
                                    f"_alg{algorithm}"
                                    f"_ndel{NUM_DELIVERIES}"
                                    f"_load{drone_load}"
                                    f"_batt{drone_battery}"
                                    f"_maxw{max_weight}"
                                    f"_maxint{max_interval_len}"
                                    f"_regsp{DEFAULT_REGULARLY_SPACED}"
                                    f"_delta{deliveries_starting_point}"
                                    f"_zipf{zipf_exponent}"
                                )

                                cmd = (
                                    f"OMP_NUM_THREADS=98 ./{BUILD_DIR}/{BIN_FILE} --params "
                                    f"-exp_name {exp_name} "
                                    f"-log {DEFAULT_LOG} "
                                    f"-seed {seed} "
                                    f"-num_deliveries {NUM_DELIVERIES} "
                                    f"-drone_battery {drone_battery} "
                                    f"-algorithm {algorithm} "
                                    f"-max_weight {max_weight} "
                                    f"-drone_load {drone_load} "
                                    f"-max_len_road {DEFAULT_MAX_LEN_ROAD} "
                                    f"-max_interval_len {max_interval_len} "
                                    f"-max_profit {DEFAULT_MAX_PROFIT} "
                                    f"-height {DEFAULT_HEIGHT} "
                                    f"-distance {DEFAULT_DISTANCE} "
                                    f"-iterations {DEFAULT_ITERATIONS} "
                                    f"-save {DEFAULT_SAVE} "
                                    f"-solution_space {solution_space} "
                                    f"-energy_unit_cost {DEFAULT_ENERGY_UNIT_COST} "
                                    f"-energy_per_delivery {energy_per_delivery} "
                                    f"-regularly_spaced {DEFAULT_REGULARLY_SPACED} "
                                    f"-deliveries_starting_point {deliveries_starting_point} "
                                    f"-error {DEFAULT_ERROR} "
                                    f"-exponent {zipf_exponent} "
                                    f"-instance_type {INSTANCE_TYPE} "
                                )

                                print(f"Executing: {cmd}")
                                try:
                                    subprocess.run(cmd, shell=True, check=True, text=True)
                                except subprocess.CalledProcessError as e:
                                    print(f"Error: Command failed with exit code {e.returncode}")
                                    raise SystemExit(1)

                                run_count += 1

print(f"Completed {run_count} runs.")

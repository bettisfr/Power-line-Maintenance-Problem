import subprocess
import os

# Parameters
BUILD_DIR = 'cmake-build-release'
BIN_FILE = 'dcoss'

os.makedirs('output', exist_ok=True)

# Default parameter values
DEFAULT_LOG = 0
DEFAULT_ITERATIONS = 5
DEFAULT_MAX_LEN_ROAD = 100
DEFAULT_MAX_PROFIT = 10
DEFAULT_HEIGHT = 0.05
DEFAULT_DISTANCE = 0.5
DEFAULT_ENERGY_UNIT_COST = 150
DEFAULT_SAVE = 1
DEFAULT_ERROR = 0.05 # 5% error

########################################################################################################################
# Variable parameters
ENERGY_PER_DELIVERY_VEC = [0, 30] # 0, 30
MAX_WEIGHT_VEC = [1, 5] # 1, 5
NUM_DELIVERIES_VEC = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100]
# NUM_DELIVERIES_VEC = [10, 20, 30, 40, 50]
DRONE_LOAD_VEC = [5, 10]
DRONE_BATTERY_VEC = [2500, 5000]
ALGORITHMS_VEC = [0, 2, 3, 6, 8] # 2, 3, 6
ZIPF_EXPONENT_VEC = [0] # 1, 2
EXHAUSTIVE = 0  # 1 = exhaustive, 0 = DP
########################################################################################################################
DEFAULT_REGULARLY_SPACED = 1 # 0 = randomly, 1 = regularly spaced
MAX_INTERVAL_LEN_VEC = [2, 4]  # in km
DELIVERIES_STARTING_POINT_VEC = [0.5, 1.0]  # in km
########################################################################################################################

# Seed initialization
seed = 0

# Loop through all parameter combinations
for energy_per_delivery in ENERGY_PER_DELIVERY_VEC:
    for max_weight in MAX_WEIGHT_VEC:

        # Determine solution space
        if EXHAUSTIVE == 1:
            STR_SOLUTION_SPACE = "exhaustive"
            SOLUTION_SPACE = 0 if energy_per_delivery == 0 else 3
        else:
            STR_SOLUTION_SPACE = "dp"
            SOLUTION_SPACE = 1 if energy_per_delivery == 0 else 4

        # Determine problem type string
        if max_weight == 1:
            STR_PROB = "tmp-u" if energy_per_delivery == 0 else "tmp-ud"
        else:
            STR_PROB = "tmp-a" if energy_per_delivery == 0 else "tmp-ad"

        for zipf_exponent in ZIPF_EXPONENT_VEC:
            for num_deliveries in NUM_DELIVERIES_VEC:
                for drone_battery in DRONE_BATTERY_VEC:
                    for drone_load in DRONE_LOAD_VEC:
                        for max_interval_len in MAX_INTERVAL_LEN_VEC:
                            for deliveries_starting_point in DELIVERIES_STARTING_POINT_VEC:
                                for algorithm in ALGORITHMS_VEC:

                                    exp_name = (
                                        f"out_{STR_PROB}"
                                        f"_{STR_SOLUTION_SPACE}"
                                        f"_alg{algorithm}"
                                        f"_ndel{num_deliveries}"
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
                                        f"-num_deliveries {num_deliveries} "
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
                                        f"-solution_space {SOLUTION_SPACE} "
                                        f"-energy_unit_cost {DEFAULT_ENERGY_UNIT_COST} "
                                        f"-energy_per_delivery {energy_per_delivery} "
                                        f"-regularly_spaced {DEFAULT_REGULARLY_SPACED} "
                                        f"-deliveries_starting_point {deliveries_starting_point} "
                                        f"-error {DEFAULT_ERROR} "
                                        f"-exponent {zipf_exponent} "
                                    )

                                    print(f"Executing: {cmd}")
                                    try:
                                        subprocess.run(cmd, shell=True, check=True, text=True)
                                    except subprocess.CalledProcessError as e:
                                        print(f"Error: Command failed with exit code {e.returncode}")
                                        exit(1)

                                seed += 1

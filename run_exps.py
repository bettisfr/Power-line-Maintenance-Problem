import subprocess
import os

# Parameters
BUILD_DIR = 'cmake-build-release'
BIN_FILE = 'dcoss'

os.makedirs('output', exist_ok=True)

# Default parameter values
DEFAULT_LOG = 0
DEFAULT_ITERATIONS = 33
DEFAULT_MAX_LEN_ROAD = 100
DEFAULT_MAX_INTERVAL_LEN = 8
DEFAULT_MAX_PROFIT = 10
DEFAULT_HEIGHT = 0.05
DEFAULT_DISTANCE = 0.5
DEFAULT_ENERGY_UNIT_COST = 150
DEFAULT_SAVE = 1

########################################################################################################################
# Parameter vectors
DEFAULT_ENERGY_PER_DELIVERY = 30 # either 0 or 30
NUM_DELIVERIES_VEC = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100]
DRONE_LOAD_VEC = [5, 10]
DRONE_BATTERY_VEC = [2500, 5000]
# ALGORITHMS = [0, 2, 3, 6]
ALGORITHMS = [0]

# Other "variable" parameters
MAX_WEIGHT = 5 # unitary if 1, arbitrary otherwise > 1 (like 5)
EXHAUSTIVE = 0 # 1 exhaustive, 0 smart way
########################################################################################################################

if EXHAUSTIVE == 1: # exhaustive
    STR_SOLUTION_SPACE = "exhaustive"
    if DEFAULT_ENERGY_PER_DELIVERY == 0: # no additional cost
        SOLUTION_SPACE = 0
    else: # with an additional cost
        SOLUTION_SPACE = 3
else: # smart way
    STR_SOLUTION_SPACE = "dp"
    if DEFAULT_ENERGY_PER_DELIVERY == 0: # no additional cost
        SOLUTION_SPACE = 1
    else: # with an additional cost
        SOLUTION_SPACE = 4

if MAX_WEIGHT == 1: # unitary
    if DEFAULT_ENERGY_PER_DELIVERY == 0: # no additional cost
        STR_PROB = "tmp-u"
    else: # with an additional cost
        STR_PROB = "tmp-ud"
else: # arbitrary
    if DEFAULT_ENERGY_PER_DELIVERY == 0: # no additional cost
        STR_PROB = "tmp-a"
    else: # with an additional cost
        STR_PROB = "tmp-ad"

DEFAULT_REGULARLY_SPACED = 0
DEFAULT_DELIVERIES_STARTING_POINT = 1 # this applies is DEFAULT_REGULARLY_SPACED = 1
DEFAULT_ERROR = 0.1 # this applies is DEFAULT_REGULARLY_SPACED = 1
DEFAULT_EXPONENT = 0 # either 0 (uniform), or 1 (normal zipf), or 2 (skewed)

# Seed initialization
seed = 0

# Loop through all parameter combinations
for num_deliveries in NUM_DELIVERIES_VEC:
    for drone_battery in DRONE_BATTERY_VEC:
        for drone_load in DRONE_LOAD_VEC:
            # The seed MUST be the same for each algorithm!

            for algorithm in ALGORITHMS:

                # if algorithm == 0:
                #     if num_deliveries > 40:
                #         continue

                exp_name = f"out_{STR_PROB}_{STR_SOLUTION_SPACE}_alg{algorithm}_ndel{num_deliveries}_load{drone_load}_batt{drone_battery}_maxw{MAX_WEIGHT}"
                cmd = (
                    f"./{BUILD_DIR}/{BIN_FILE} --params "
                    f"-exp_name {exp_name} "
                    f"-log {DEFAULT_LOG} "
                    f"-seed {seed} "
                    f"-num_deliveries {num_deliveries} "
                    f"-drone_battery {drone_battery} "
                    f"-algorithm {algorithm} "
                    f"-max_weight {MAX_WEIGHT} "
                    f"-drone_load {drone_load} "
                    f"-max_len_road {DEFAULT_MAX_LEN_ROAD} "
                    f"-max_interval_len {DEFAULT_MAX_INTERVAL_LEN} "
                    f"-max_profit {DEFAULT_MAX_PROFIT} "
                    f"-height {DEFAULT_HEIGHT} "
                    f"-distance {DEFAULT_DISTANCE} "
                    f"-iterations {DEFAULT_ITERATIONS} "
                    f"-save {DEFAULT_SAVE} "
                    f"-solution_space {SOLUTION_SPACE} "
                    f"-energy_unit_cost {DEFAULT_ENERGY_UNIT_COST} "
                    f"-energy_per_delivery {DEFAULT_ENERGY_PER_DELIVERY} "
                    f"-regularly_spaced {DEFAULT_REGULARLY_SPACED} "
                    f"-deliveries_starting_point {DEFAULT_DELIVERIES_STARTING_POINT} "
                    f"-error {DEFAULT_ERROR} "
                    f"-exponent {DEFAULT_EXPONENT} "
                )

                print(f"Executing: {cmd}")
                try:
                    # Execute the command
                    result = subprocess.run(cmd, shell=True, check=True, text=True)
                    #print(f"Command completed successfully")
                except subprocess.CalledProcessError as e:
                    print(f"Error: Command failed with exit code {e.returncode}")
                    exit(1)

            # Increment the seed
            seed += 1

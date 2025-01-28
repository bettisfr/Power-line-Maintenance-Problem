import subprocess
import os

# Parameters
BUILD_DIR = 'cmake-build-release'
BIN_FILE = 'dcoss'

os.makedirs('output', exist_ok=True)

# Parameter vectors
NUM_DELIVERIES_VEC = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100]
# NUM_DELIVERIES_VEC = [100]
MAX_WEIGHT_VEC = [1, 5]
DRONE_LOAD_VEC = [5, 10]
DRONE_BATTERY_VEC = [2500, 5000]
ALGORITHMS = [2, 3, 6, 8, 0]
#ALGORITHMS = [0]

# Default parameter values
DEFAULT_LOG = 0
DEFAULT_ITERATIONS = 33
DEFAULT_MAX_LEN_ROAD = 50
DEFAULT_MAX_INTERVAL_LEN = 2
DEFAULT_MAX_PROFIT = 10
DEFAULT_HEIGHT = 0.5
DEFAULT_DISTANCE = 0.5
DEFAULT_ENERGY_UNIT_COST = 200
DEFAULT_ENERGY_PER_DELIVERY = 0
DEFAULT_SOLUTION_SPACE = 1
DEFAULT_SAVE = 1

# Seed initialization
seed = 0

# Loop through all parameter combinations
for num_deliveries in NUM_DELIVERIES_VEC:
    for drone_battery in DRONE_BATTERY_VEC:
        for max_weight in MAX_WEIGHT_VEC:
            for drone_load in DRONE_LOAD_VEC:
                # The seed MUST be the same for each algorithm!

                for algorithm in ALGORITHMS:

                    if algorithm == 0:
                        if num_deliveries > 150:
                            continue

                    exp_name = f"out_alg{algorithm}_ndel{num_deliveries}_maxw{max_weight}_load{drone_load}_batt{drone_battery}"
                    cmd = (
                        f"./{BUILD_DIR}/{BIN_FILE} --params "
                        f"-exp_name {exp_name} "
                        f"-log {DEFAULT_LOG} "
                        f"-seed {seed} "
                        f"-num_deliveries {num_deliveries} "
                        f"-drone_battery {drone_battery} "
                        f"-algorithm {algorithm} "
                        f"-max_weight {max_weight} "
                        f"-drone_load {drone_load} "
                        f"-max_len_road {DEFAULT_MAX_LEN_ROAD} "
                        f"-max_interval_len {DEFAULT_MAX_INTERVAL_LEN} "
                        f"-max_profit {DEFAULT_MAX_PROFIT} "
                        f"-height {DEFAULT_HEIGHT} "
                        f"-distance {DEFAULT_DISTANCE} "
                        f"-iterations {DEFAULT_ITERATIONS} "
                        f"-save {DEFAULT_SAVE} "
                        f"-solution_space {DEFAULT_SOLUTION_SPACE} "
                        f"-energy_unit_cost {DEFAULT_ENERGY_UNIT_COST} "
                        f"-energy_per_delivery {DEFAULT_ENERGY_PER_DELIVERY} "
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

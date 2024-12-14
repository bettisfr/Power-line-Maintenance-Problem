#!/bin/bash

BUILD_DIR='cmake-build-release'
BIN_FILE='dcoss'

# Parameter vectors
#NUM_DELIVERIES_VEC=(50 100 200)
NUM_DELIVERIES_VEC=(5 10 20)
MAX_WEIGHT_VEC=(1 5)
DRONE_LOAD_VEC=(5 10)
DRONE_BATTERY_VEC=(2500 5000)
ALGORITHMS=(0 1 2 3 4 5 6 7 8)

# Default parameter values
DEFAULT_LOG=0
DEFAULT_ITERATIONS=3
DEFAULT_MAX_LEN_ROAD=100
DEFAULT_MAX_INTERVAL_LEN=15
DEFAULT_MAX_PROFIT=10
DEFAULT_HEIGHT=0.5
DEFAULT_ENERGY_UNIT_COST=200
DEFAULT_SAVE=1

SEED=0

for NUM_DELIVERIES in "${NUM_DELIVERIES_VEC[@]}"; do
    for ALGORITHM in "${ALGORITHMS[@]}"; do
        for MAX_WEIGHT in "${MAX_WEIGHT_VEC[@]}"; do
            for DRONE_LOAD in "${DRONE_LOAD_VEC[@]}"; do
                for DRONE_BATTERY in "${DRONE_BATTERY_VEC[@]}"; do
                    EXP_NAME="out_alg${ALGORITHM}_ndel${NUM_DELIVERIES}_maxw${MAX_WEIGHT}_load${DRONE_LOAD}_batt${DRONE_BATTERY}"
                    CMD="./${BUILD_DIR}/${BIN_FILE} --params \
                        -exp_name $EXP_NAME \
                        -log $DEFAULT_LOG \
                        -seed $SEED \
                        -num_deliveries $NUM_DELIVERIES \
                        -drone_battery $DRONE_BATTERY \
                        -algorithm $ALGORITHM \
                        -max_weight $MAX_WEIGHT \
                        -drone_load $DRONE_LOAD \
                        -max_len_road $DEFAULT_MAX_LEN_ROAD \
                        -max_interval_len $DEFAULT_MAX_INTERVAL_LEN \
                        -max_profit $DEFAULT_MAX_PROFIT \
                        -height $DEFAULT_HEIGHT \
                        -iterations $DEFAULT_ITERATIONS \
                        -save $DEFAULT_SAVE \
                        -energy_unit_cost $DEFAULT_ENERGY_UNIT_COST"
                    CMD=$(echo "$CMD" | tr -s ' ')  # Replace multiple spaces with a single space
                    echo "Executing: $CMD"

                    # Execute the command
                    if ! eval "$CMD"; then
                        echo "Error: Command failed. Exiting."
                        exit 1
                    fi

                    ((SEED++))
                done
            done
        done
    done
done

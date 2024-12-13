#!/bin/bash

BUILD_DIR='cmake-build-release'
BIN_FILE='dcoss'

NUM_DELIVERIES_VEC=(10 20 30 40)
DRONE_BATTERY_VEC=(100 200)

ITERATIONS=33
SEED=0

for NUM_DELIVERIES in "${NUM_DELIVERIES_VEC[@]}"; do
    for DRONE_BATTERY in "${DRONE_BATTERY_VEC[@]}"; do
        EXP_NAME="out_n${NUM_DELIVERIES}_b${DRONE_BATTERY}"
        CMD="./${BUILD_DIR}/${BIN_FILE} --params -exp_name $EXP_NAME -seed $SEED -num_deliveries $NUM_DELIVERIES -drone_battery $DRONE_BATTERY -iterations $ITERATIONS -save 1"
        echo "Executing: $CMD"
        $CMD

        ((SEED++))
    done
done

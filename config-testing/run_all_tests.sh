#!/bin/bash

if [ $# -eq 0 ]; then
    BAG_INFO="./pioneer3at-realsense-bag1.yaml"
elif [ $# -eq 1 ]; then
    BAG_INFO=$1
else
    echo "invalid number of arguments"
    exit 1
fi

mkdir -p test-results # create dir if it doesn't exist already

for TEST_FILE in tests/*; do
    LOGFILE="./test-results/$(basename $TEST_FILE .yaml).log"
    python3 test_localization_configs.py \
        --base_config ./base_localization_config.yaml \
        --sensor_data_bag_info $BAG_INFO \
        --output_dir ./test-results/ \
        --test_config $TEST_FILE 2>&1 | tee $LOGFILE
done
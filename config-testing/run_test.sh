#!/bin/bash

if [ $# -eq 1 ]; then
    TEST_CONFIG_FILE=$1
else
    echo "invalid number of arguments"
    exit 1
fi

mkdir -p test-results # create dir if it doesn't exist already

BAG_INFO="./pioneer3at-realsense-bag1.yaml"

LOGFILE=".test-results/$(basename $TEST_FILE .yaml).log"
python3 test_localization_configs.py \
    --base_config ./base_localization_config.yaml \
    --sensor_data_bag_info $BAG_INFO \
    --output_dir ./test-results/ \
    --test_config $TEST_CONFIG_FILE | tee $LOGFILE
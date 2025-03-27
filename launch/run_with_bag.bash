#!/bin/bash

dir=$(dirname $0)/..

( sleep 5 && ros2 bag play $dir/bag/rosbag2_2025_01_22-13_29_26 ) &

ros2 launch flow_estimator ogm_flow_estimator_static.launch.py

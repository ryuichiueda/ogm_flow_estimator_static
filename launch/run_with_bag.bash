#!/bin/bash

dir=$(dirname $0)/..

( sleep 5 && ros2 bag play $dir/bag/rosbag2_2025_01_22-13_29_26 ) &
cargo run --release

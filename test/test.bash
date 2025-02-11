#!/bin/bash

dir=~
[ "$1" != "" ] && dir="$1"   #引数があったら、そちらをホームに変える。

source $dir/.bashrc
cd $dir/ros2_ws
colcon build
source $dir/.bashrc

cd $dir/ros2_ws/src/ogm_flow_estimator_static
cargo build --release

( sleep 5 && ros2 bag play ./bag/rosbag2_2025_01_22-13_29_26 ) &
timeout 30 cargo run --release |& tee - log.txt


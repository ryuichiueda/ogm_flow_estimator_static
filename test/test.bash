#!/bin/bash

dir=~
[ "$1" != "" ] && dir="$1"   #引数があったら、そちらをホームに変える。

cd $dir/ros2_ws
colcon build
source $dir/.bashrc

cd $dir/ros2_ws/src/ogm_flow_estimator_static
cargo build --release

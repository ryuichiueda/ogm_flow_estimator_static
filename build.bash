#!/bin/bash

pkg=ogm_flow_estimator_static
dir=$(dirname $0)

cargo build
launch_dir=$dir/../../install/$pkg/share/$pkg/launch/
mkdir -p $launch_dir
cp $dir/launch/* $launch_dir/

# Occupancy Grid Map Flow Estimator (Static Version)

Concise motion estimator on an occupied grid map with a static viewpoint, whose basic algorithm has been presented on a domestic conference. 
(I will present it in some international conference or journal with some improvement.)
This package is available on ROS 2 Humble. 

[!['demo'](http://img.youtube.com/vi/FvutzPRR0Qc/sddefault.jpg)](https://www.youtube.com/watch?v=FvutzPRR0Qc)

## How to use

This is a ROS 2 repository using [ros2_rust](https://github.com/ros2-rust/ros2_rust). The way of installation and execution are based on the manual or `ros2_rust`. They may be different from the ways of regular (Python or C++) packages. 

### node and its interfaces

As the author is not used to `ros2_rust`, parameters and names of topics are fixed at present.

#### `flow_estimator`

* topic for subscribing
    * `/scan` (`sensor_msgs/LaserScan`): scan data from 2D LiDAR
* topic for publising
    * `/scan_map` (`nav_msgs/OccupancyGrid`): an occupancy grid map simply reflecting the latest scan
    * `/static_obstacle_map` (`nav_msgs/OccupancyGrid`): an occupancy grid map of static objects
    * `/estimaton_array` (`visualization_msgs/MarkerArray`): estimation results (vectors)
        * vector
            * start: current position of a part of an obstacle
            * end: forecast of the position after one second

## Required software

* ROS 2 Humble: https://docs.ros.org/en/humble/index.html
* ros2_rust: https://github.com/ros2-rust/ros2_rust

## launch

There is no launch files since I have never understood how to write them for ros2_rust packages. Instead, I placed a bash script in `launch` directory. This script builds and runs `flow_estimator` node. Moreover, it plays a bag file which contains messages on `/scan` topic. So as you launch `rviz2` from another terminal, you can observe messages on the `estimaton_array` topic. 

## References

* https://www.docswell.com/s/ryuichiueda/ZEX11D-si2024

© 2025 Ryuichi Ueda

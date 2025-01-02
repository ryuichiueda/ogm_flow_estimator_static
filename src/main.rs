//SPDX-FileCopyrightText: ros2_rust contributers
//SPDX-FileCopyrightText: Ryuichi Ueda <ryuichiueda@gmail.com>
//SPDX-License-Identifier: BSD-3-Clause

mod map;

use std::sync::{Arc, Mutex};
use sensor_msgs::msg::LaserScan;
use nav_msgs::msg::OccupancyGrid;
use map::StaticObstacleMap;
use std::ops::Deref;

struct FlowEstimatorNode {
    node: Arc<rclrs::Node>,
    _sub_scan: Arc<rclrs::Subscription<LaserScan>>,
    data: Arc<Mutex<Option<LaserScan>>>,
    static_obstacle_map: Arc<rclrs::Publisher<OccupancyGrid>>,
}

impl FlowEstimatorNode {
    fn new(context: &rclrs::Context) -> Result<Self, rclrs::RclrsError> {
        let node = rclrs::Node::new(context, "flow_estimator")?;
        let data = Arc::new(Mutex::new(None));
        let data_cb = Arc::clone(&data);
        let _sub_scan = node.create_subscription(
            "scan",
            rclrs::QOS_PROFILE_DEFAULT,
            move |msg: LaserScan| { *data_cb.lock().unwrap() = Some(msg); },
        )?;

        let static_obstacle_map = node.create_publisher("static_obstacle_map", rclrs::QOS_PROFILE_DEFAULT)?;
        Ok(Self {
            node,
            _sub_scan,
            data,
            static_obstacle_map,
        })
    }

    fn republish(&self, static_obs_map: &mut StaticObstacleMap) -> Result<(), rclrs::RclrsError> {
        let scan = match self.data.lock().unwrap().deref() {
            Some(s) => s.clone(),
            None => {
                eprintln!("waiting scan");
                return Ok(());
            },
        };

        static_obs_map.scan_to_occupancy(&scan);
        self.static_obstacle_map.publish(static_obs_map.map.clone())?;

        Ok(())
    }
}

fn main() -> Result<(), rclrs::RclrsError> {
    let context = rclrs::Context::new(std::env::args())?;
    let republisher = Arc::new(FlowEstimatorNode::new(&context)?);
    let republisher_other_thread = Arc::clone(&republisher);

    std::thread::spawn(move || -> Result<(), rclrs::RclrsError> {
        let mut static_obs_map = StaticObstacleMap::new(120, 120, 0.1);

        loop {
            use std::time::Duration;
            std::thread::sleep(Duration::from_millis(1000));
            republisher_other_thread.republish(&mut static_obs_map)?;
        }
    });

    rclrs::spin(Arc::clone(&republisher.node))
}

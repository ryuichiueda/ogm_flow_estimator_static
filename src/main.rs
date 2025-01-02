//SPDX-FileCopyrightText: ros2_rust contributers
//SPDX-FileCopyrightText: Ryuichi Ueda <ryuichiueda@gmail.com>
//SPDX-License-Identifier: BSD-3-Clause

mod map;

use std::sync::{Arc, Mutex};
use sensor_msgs::msg::LaserScan;
use nav_msgs::msg::OccupancyGrid;

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

    fn republish(&self) -> Result<(), rclrs::RclrsError> {
        let scan = self.data.lock().unwrap();

        let mut map = map::generate(60, 120, 0.1);
        map::plot(1.0, 2.0, 100, &mut map);

        self.static_obstacle_map.publish(map)?;

        Ok(())
    }
}

fn main() -> Result<(), rclrs::RclrsError> {
    let context = rclrs::Context::new(std::env::args())?;
    let republisher = Arc::new(FlowEstimatorNode::new(&context)?);
    let republisher_other_thread = Arc::clone(&republisher);

    std::thread::spawn(move || -> Result<(), rclrs::RclrsError> {
        loop {
            use std::time::Duration;
            std::thread::sleep(Duration::from_millis(1000));
            republisher_other_thread.republish()?;
        }
    });

    rclrs::spin(Arc::clone(&republisher.node))
}

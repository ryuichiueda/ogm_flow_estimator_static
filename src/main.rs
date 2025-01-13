//SPDX-FileCopyrightText: ros2_rust contributers
//SPDX-FileCopyrightText: Ryuichi Ueda <ryuichiueda@gmail.com>
//SPDX-License-Identifier: BSD-3-Clause

mod map;
mod scan_map;
mod static_map;
mod dynamic_map;

use std::sync::{Arc, Mutex};
use sensor_msgs::msg::LaserScan;
use nav_msgs::msg::OccupancyGrid;
use std::ops::Deref;

struct FlowEstimatorNode {
    node: Arc<rclrs::Node>,
    _sub_scan: Arc<rclrs::Subscription<LaserScan>>,
    data: Arc<Mutex<Option<LaserScan>>>,
    scan_map: Arc<rclrs::Publisher<OccupancyGrid>>,
    static_obstacle_map: Arc<rclrs::Publisher<OccupancyGrid>>,
    dynamic_obstacle_map: Arc<rclrs::Publisher<OccupancyGrid>>,
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

        let scan_map = node.create_publisher("scan_map", rclrs::QOS_PROFILE_DEFAULT)?;
        let static_obstacle_map = node.create_publisher("static_obstacle_map", rclrs::QOS_PROFILE_DEFAULT)?;
        let dynamic_obstacle_map = node.create_publisher("dynamic_obstacle_map", rclrs::QOS_PROFILE_DEFAULT)?;
        Ok(Self {
            node,
            _sub_scan,
            data,
            scan_map,
            static_obstacle_map,
            dynamic_obstacle_map,
        })
    }

    fn publish_scan_map(&self, buffer: &mut Vec<OccupancyGrid>) -> Result<(), rclrs::RclrsError> {
        let scan = match self.data.lock().unwrap().deref() {
            Some(s) => s.clone(),
            None => {
                eprintln!("waiting scan");
                return Ok(());
            },
        };

        if let Some(last_map) = buffer.last() {
            if last_map.info.map_load_time == scan.header.stamp {
                return Ok(());
            }
        }

        let map = scan_map::generate(120, 120, 0.1, &scan);
        buffer.push(map.clone());
        self.scan_map.publish(map)?;

        if buffer.len() > 10 {
            buffer.remove(0);
        }

        Ok(())
    }

    fn publish_static_obstacle_map(&self, map: &OccupancyGrid) -> Result<(), rclrs::RclrsError> {
        self.static_obstacle_map.publish(map)?;
        Ok(())
    }

    fn publish_dynamic_obstacle_map(&self, map: &OccupancyGrid) -> Result<(), rclrs::RclrsError> {
        self.dynamic_obstacle_map.publish(map)?;
        Ok(())
    }
}

fn main() -> Result<(), rclrs::RclrsError> {
    let context = rclrs::Context::new(std::env::args())?;
    let republisher = Arc::new(FlowEstimatorNode::new(&context)?);
    let republisher_other_thread = Arc::clone(&republisher);

    let mut map_buffer = vec![];

    std::thread::spawn(move || -> Result<(), rclrs::RclrsError> {

        loop {
            use std::time::Duration;
            std::thread::sleep(Duration::from_millis(200));
            republisher_other_thread.publish_scan_map(&mut map_buffer)?;

            if let Some(static_map) = static_map::generate(&mut map_buffer) {
                republisher_other_thread.publish_static_obstacle_map(&static_map)?;
            }

            if let Some(dynamic_map) = dynamic_map::generate(&mut map_buffer) {
                republisher_other_thread.publish_dynamic_obstacle_map(&dynamic_map)?;
            }
        }
    });

    rclrs::spin(Arc::clone(&republisher.node))
}

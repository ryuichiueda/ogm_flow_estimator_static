//SPDX-FileCopyrightText: ros2_rust contributers
//SPDX-FileCopyrightText: Ryuichi Ueda <ryuichiueda@gmail.com>
//SPDX-License-Identifier: BSD-3-Clause

mod estimator;
mod map;
mod scan_map;
mod static_map;

use crate::estimator::Estimator;
use std::sync::{Arc, Mutex};
use sensor_msgs::msg::LaserScan;
use nav_msgs::msg::OccupancyGrid;
use visualization_msgs::msg::MarkerArray;
use std::ops::Deref;

struct FlowEstimatorNode {
    node: Arc<rclrs::Node>,
    _sub_scan: Arc<rclrs::Subscription<LaserScan>>,
    data: Arc<Mutex<Option<LaserScan>>>,
    scan_map: Arc<rclrs::Publisher<OccupancyGrid>>,
    static_obstacle_map: Arc<rclrs::Publisher<OccupancyGrid>>,
    obstacle_motion: Arc<rclrs::Publisher<MarkerArray>>,
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
        let obstacle_motion = node.create_publisher("estimation_array", rclrs::QOS_PROFILE_DEFAULT)?;
        Ok(Self {
            node,
            _sub_scan,
            data,
            scan_map,
            static_obstacle_map,
            obstacle_motion,
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

        if buffer.len() > 100 {
            buffer.remove(0);
        }

        Ok(())
    }

    fn publish_static_obstacle_map(&self, map: &OccupancyGrid) -> Result<(), rclrs::RclrsError> {
        self.static_obstacle_map.publish(map)?;
        Ok(())
    }

    fn publish_obstacle_motion(&self, map: &MarkerArray) -> Result<(), rclrs::RclrsError> {
        self.obstacle_motion.publish(map)?;
        Ok(())
    }
}

fn main() -> Result<(), rclrs::RclrsError> {
    let context = rclrs::Context::new(std::env::args())?;
    let republisher = Arc::new(FlowEstimatorNode::new(&context)?);
    let republisher_other_thread = Arc::clone(&republisher);

    let mut map_buffer = vec![];

    std::thread::spawn(move || -> Result<(), rclrs::RclrsError> {
        let mut estimator = Estimator::new();

        loop {
            use std::time::Duration;
            std::thread::sleep(Duration::from_millis(200));
            republisher_other_thread.publish_scan_map(&mut map_buffer)?;

            if let Some(static_map) = static_map::generate(&mut map_buffer) {
                republisher_other_thread.publish_static_obstacle_map(&static_map)?;
                match estimator.generate(&mut map_buffer, &static_map) {
                    Ok(Some(estimation)) => 
                        republisher_other_thread.publish_obstacle_motion(&estimation)?,
                    Ok(None) => eprintln!("waiting ..."),
                    Err(e)   => eprintln!("{:?}", &e),
                }
            }
        }
    });

    rclrs::spin(Arc::clone(&republisher.node))
}

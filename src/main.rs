//SPDX-FileCopyrightText: ros2_rust contributers
//SPDX-FileCopyrightText: Ryuichi Ueda <ryuichiueda@gmail.com>
//SPDX-License-Identifier: BSD-3-Clause

use std::sync::{Arc, Mutex};
use sensor_msgs::msg::LaserScan;
use nav_msgs::msg::OccupancyGrid;
use nav_msgs::msg::MapMetaData;
use std_msgs::msg::Header;
use rclrs::Clock;
use builtin_interfaces::msg::Time;
use geometry_msgs::msg::Pose;

struct FlowEstimatorNode {
    node: Arc<rclrs::Node>,
    _sub_scan: Arc<rclrs::Subscription<LaserScan>>,
    data: Arc<Mutex<Option<LaserScan>>>,
    obstacle_map: Arc<rclrs::Publisher<OccupancyGrid>>,
}

impl FlowEstimatorNode {
    fn new(context: &rclrs::Context) -> Result<Self, rclrs::RclrsError> {
        let node = rclrs::Node::new(context, "republisher")?;
        let data = Arc::new(Mutex::new(None));
        let data_cb = Arc::clone(&data);
        let _sub_scan = node.create_subscription(
            "scan",
            rclrs::QOS_PROFILE_DEFAULT,
            move |msg: LaserScan| { *data_cb.lock().unwrap() = Some(msg); },
        )?;

        let obstacle_map = node.create_publisher("obstacle_map", rclrs::QOS_PROFILE_DEFAULT)?;
        Ok(Self {
            node,
            _sub_scan,
            data,
            obstacle_map,
        })
    }

    fn republish(&self) -> Result<(), rclrs::RclrsError> {
        let scan = self.data.lock().unwrap();
        dbg!("{:?}", &scan);

        let clock = Clock::system();
        let now = clock.now();
        let nanosec = (now.nsec as u32 ) % 1_000_000_000;
        let sec = (now.nsec / 1_000_000_000) as i32;

        let header = Header {
            stamp: Time{ nanosec, sec },
            frame_id: "map".to_string(),
        };

        let info = MapMetaData {
            map_load_time: Time{ nanosec: nanosec, sec: sec },
            resolution: 0.1,
            width: 13, 
            height: 13, 
            origin: Pose::default(),
        };

        let mut tmp = vec![100; 13*13];
        let black = vec![1, 4, 5, 9, 10, 11,
                         0+13, 1+13, 2+13, 11+13,
                         0+26, 2+26, 7+26, 11+26,
                         2+39, 7+39, 11+39,
                         2+52, 4+52, 5+52, 6+52, 9+52, 10+52, 11+52,
                         0+78, 1+78, 6+78, 7+78, 8+78,
                         0+91, 7+91,
                         0+104, 1+104, 3+104, 5+104, 6+104, 7+104, 8+104, 10+104, 12+104,
                         0+117, 3+117, 5+117, 7+117, 10+117, 12+117,
                         0+130, 5+130, 7+130, 12+130,
                         0+143, 1+143, 2+143, 5+143, 7+143, 8+143, 9+143, 12+143,
                         4+156, 5+156, 11+156
        ];
        for b in black {
            tmp[b] = 0;
        }

        let mut data = vec![];
        for i in 0..13 {
            let v = tmp[i*13..i*13+13].to_vec();
            data.append( &mut v.into_iter().rev().collect() );
        }

        let map = OccupancyGrid { header, info, data, };
        self.obstacle_map.publish(map)?;

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

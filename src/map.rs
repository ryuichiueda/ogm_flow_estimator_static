//SPDX-FileCopyrightText: Ryuichi Ueda <ryuichiueda@gmail.com>
//SPDX-License-Identifier: BSD-3-Clause

use nav_msgs::msg::OccupancyGrid;
use nav_msgs::msg::MapMetaData;
use std_msgs::msg::Header;
use rclrs::Clock;
use builtin_interfaces::msg::Time;
use geometry_msgs::msg::Pose;

pub fn generate(width: u32, height: u32, resolution: f32) -> OccupancyGrid {
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
            resolution,
            width, 
            height, 
            origin: Pose::default(),
        };

        let size = (width*height) as usize;
        OccupancyGrid { header, info, data: vec![0; size], }
}

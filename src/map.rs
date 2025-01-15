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

    let mut origin = Pose::default();
    origin.position.x = -((width as f32)*resolution/2.0) as f64;
    origin.position.y = -((height as f32)*resolution/2.0) as f64;

    let header = Header {
        stamp: Time{ nanosec, sec },
        frame_id: "base_scan".to_string(),
    };

    let info = MapMetaData {
        map_load_time: Time{ nanosec: nanosec, sec: sec },
        resolution,
        width, 
        height, 
        origin,
    };

    let size = (width*height) as usize;
    OccupancyGrid { header, info, data: vec![0; size], }
}

pub fn time_diff(from: &Time ,to: &Time) -> f64 {
    let sec_diff = to.sec as f64 - from.sec as f64;
    let nanosec_diff = to.nanosec as f64 - from.nanosec as f64;
    sec_diff + nanosec_diff/1_000_000_000.0
}

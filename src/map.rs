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

pub fn ixiy_to_index(ix: i32, iy: i32, width: u32, height: u32) -> Option<usize> {
    if ix < 0 || ix >= width as i32 || iy < 0 || iy >= height as i32 {
        return None;
    }

    Some(iy as usize * width as usize + ix as usize)
}

pub fn index_to_ixiy(index: usize, width: u32, height: u32) -> Option<(i32, i32)> {
    let ix: i32 = (index as i32)%(width as i32);
    let iy: i32 = (index as i32)/(width as i32);

    if ix < 0 || ix >= width as i32 || iy < 0 || iy >= height as i32 {
        return None;
    }

    Some((ix, iy))
}

pub fn index_to_real_pos(index: usize, width: u32, height: u32, resolution: f32) -> Option<(f64, f64)> {
    let (ix, iy) = index_to_ixiy(index, width, height)?;

    let x = ix as f64 - (width as f64)/2.0;
    let y = iy as f64 - (height as f64)/2.0;

    Some((x*resolution as f64, y*resolution as f64))
}

pub fn time(map: &OccupancyGrid) -> f64 {
    let sec = map.info.map_load_time.sec as f64;
    let nanosec = map.info.map_load_time.nanosec as f64;

    sec + nanosec/(1_000_000_000 as f64)
}

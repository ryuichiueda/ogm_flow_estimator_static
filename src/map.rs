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

pub struct StaticObstacleMap {
    pub map: OccupancyGrid,
}

impl StaticObstacleMap {
    pub fn new(width: u32, height: u32, resolution: f32) -> Self {
        Self {
            map: generate(width, height, resolution),
        }
    }

    pub fn plot(&mut self, x: f64, y: f64, val: i8) {
        let res = self.map.info.resolution as f64;
        let map_x = ((x - self.map.info.origin.position.x)/res).floor() as i32;
        let map_y = ((y - self.map.info.origin.position.y)/res).floor() as i32;
    
        if map_x < 0 || map_y < 0 {
            return;
        }
    
        let index = (map_y as usize)*(self.map.info.width as usize)
                    + (map_x as usize);
    
        self.map.data[index] = val;
    }
}

//SPDX-FileCopyrightText: Ryuichi Ueda <ryuichiueda@gmail.com>
//SPDX-License-Identifier: BSD-3-Clause

use builtin_interfaces::msg::Time;
use nav_msgs::msg::OccupancyGrid;

pub fn generate(buffer: &Vec<OccupancyGrid>) -> Option<OccupancyGrid> {
    if buffer.is_empty() {
        return None;
    }

    let mut last_map_time = Time{ nanosec: 0, sec: 0 };
    dbg!("-----------------------");

    for map in buffer.iter().rev() {
        if last_map_time.sec == 0 {
            last_map_time = map.info.map_load_time.clone();
        }
        
        let diff = match last_map_time.nanosec >= map.info.map_load_time.nanosec {
            true  => {
                Time{ nanosec: last_map_time.nanosec - map.info.map_load_time.nanosec,
                      sec: last_map_time.sec - map.info.map_load_time.sec }
            },
            false => {
                Time{ nanosec: 1_000_000_000 + last_map_time.nanosec - map.info.map_load_time.nanosec,
                      sec: last_map_time.sec - map.info.map_load_time.sec - 1}
            },
        };

        dbg!("{:?}", &diff);
    }
    
    let last = buffer.last()?;

    Some(last.clone())
}

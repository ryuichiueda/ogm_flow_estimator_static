//SPDX-FileCopyrightText: Ryuichi Ueda <ryuichiueda@gmail.com>
//SPDX-License-Identifier: BSD-3-Clause

use builtin_interfaces::msg::Time;
use nav_msgs::msg::OccupancyGrid;

pub fn generate(buffer: &Vec<OccupancyGrid>) -> Option<OccupancyGrid> {
    if buffer.is_empty() {
        return None;
    }

    let mut last_map_time = Time{ nanosec: 0, sec: 0 };
    for map in buffer.iter().rev() {
        if last_map_time.sec == 0 {
            last_map_time = map.info.map_load_time.clone();
        }

        dbg!("{:?}", &map.info.map_load_time);
    }
    
    let last = buffer.last()?;

    Some(last.clone())
}

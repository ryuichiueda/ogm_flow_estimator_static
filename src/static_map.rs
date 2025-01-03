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
        
        let diff = time_diff(&map.info.map_load_time, &last_map_time);

        dbg!("{:?}", &diff);
    }
    
    let last = buffer.last()?;

    Some(last.clone())
}

fn time_diff(from: &Time ,to: &Time) -> Time {
    match to.nanosec >= from.nanosec {
        true  => Time{ nanosec: to.nanosec - from.nanosec,
                       sec: to.sec - from.sec },
        false => Time{ nanosec: 1_000_000_000 + to.nanosec - from.nanosec,
                       sec: to.sec - from.sec - 1},
    }
}

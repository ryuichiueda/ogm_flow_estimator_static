//SPDX-FileCopyrightText: Ryuichi Ueda <ryuichiueda@gmail.com>
//SPDX-License-Identifier: BSD-3-Clause

use crate::map;
use nav_msgs::msg::OccupancyGrid;

pub fn generate(buffer: &Vec<OccupancyGrid>) -> Option<OccupancyGrid> {
    const LIMIT_SEC: f64 = -5.0;
    const SKIP_SEC: f64 = 1.0;
    const STATIC_OCCUPANCY_TH: i32 = 80; //percent

    let mut ans = buffer.last()?.clone();
    let last_map_time = ans.info.map_load_time.clone();

    ans.data = vec![0; (ans.info.width*ans.info.height).try_into().unwrap()];

    let mut next = 0.0;
    let mut counter = 0;
    let mut ok = false;
    for map in buffer.iter().rev() {
        let diff = map::time_diff(&last_map_time, &map.info.map_load_time);
        if diff < LIMIT_SEC {
            ok = true;
            break;
        }
        if next < diff {
            continue;
        }

        counter += 1;
        for i in 0..map.data.len() {
            if map.data[i] > 0 {
                ans.data[i] += 1;
            }
        }

        next -= SKIP_SEC;
    }

    if ! ok {
        return None;
    }

    let th = (counter*STATIC_OCCUPANCY_TH/100) as i8;
    ans.data.iter_mut().for_each(|e| *e = if *e >= th{100}else{0});
    Some(ans)
}


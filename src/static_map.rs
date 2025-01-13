//SPDX-FileCopyrightText: Ryuichi Ueda <ryuichiueda@gmail.com>
//SPDX-License-Identifier: BSD-3-Clause

use builtin_interfaces::msg::Time;
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
        let diff = time_diff(&last_map_time, &map.info.map_load_time);
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

fn time_diff(from: &Time ,to: &Time) -> f64 {
    let sec_diff = to.sec as f64 - from.sec as f64;
    let nanosec_diff = to.nanosec as f64 - from.nanosec as f64;

    sec_diff + nanosec_diff/1_000_000_000.0
    /*
    match to.nanosec >= from.nanosec {
        true  => Time{ nanosec: to.nanosec - from.nanosec,
                       sec: to.sec - from.sec },
        false => Time{ nanosec: 1_000_000_000 + to.nanosec - from.nanosec,
                       sec: to.sec - from.sec - 1},
    }
    */
}

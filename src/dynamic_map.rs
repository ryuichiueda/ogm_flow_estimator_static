//SPDX-FileCopyrightText: Ryuichi Ueda <ryuichiueda@gmail.com>
//SPDX-License-Identifier: BSD-3-Clause

use crate::map;
use nav_msgs::msg::OccupancyGrid;

pub fn generate(buffer: &Vec<OccupancyGrid>, static_map: &OccupancyGrid) -> Option<OccupancyGrid> {
    const LIMIT_SEC: f64 = -0.9;
    const SKIP: usize = 1;

    let mut ans = buffer.last()?.clone();
    let last_map_time = ans.info.map_load_time.clone();

    subtract_static(&mut ans, static_map);

    let mut count = 0;
    for (i, map) in buffer.iter().rev().enumerate() {
        if i%SKIP != 0 {
            continue;
        }

        let diff = map::time_diff(&last_map_time, &map.info.map_load_time);
        if diff < LIMIT_SEC {
            break;
        }
        dbg!("{:?}", &diff);

        count += 1;
    }

    Some(ans)
}

fn subtract_static(dynamic_map: &mut OccupancyGrid, static_map: &OccupancyGrid) {
    dynamic_map.data.iter_mut().zip(static_map.data.iter())
        .for_each(|(d, s)| if *d < *s { *d = 0; }else { *d -= *s; });
}

//SPDX-FileCopyrightText: Ryuichi Ueda <ryuichiueda@gmail.com>
//SPDX-License-Identifier: BSD-3-Clause

use nav_msgs::msg::OccupancyGrid;

pub fn generate(buffer: &Vec<OccupancyGrid>, static_map: &OccupancyGrid) -> Option<OccupancyGrid> {
    let mut ans = buffer.last()?.clone();
    subtract_static(&mut ans, static_map);
    Some(ans)
}

fn subtract_static(dynamic_map: &mut OccupancyGrid, static_map: &OccupancyGrid) {
    dynamic_map.data.iter_mut().zip(static_map.data.iter())
        .for_each(|(d, s)| if *d < *s { *d = 0; }else { *d -= *s; });
}

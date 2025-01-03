//SPDX-FileCopyrightText: Ryuichi Ueda <ryuichiueda@gmail.com>
//SPDX-License-Identifier: BSD-3-Clause

use crate::map;
use nav_msgs::msg::OccupancyGrid;

pub fn generate(buffer: &Vec<OccupancyGrid>) -> Option<OccupancyGrid> {
    if buffer.is_empty() {
        return None;
    }
    
    let last = buffer.last()?;

    Some(last.clone())
}

/*
pub struct StaticObstacleMap {
    pub map: OccupancyGrid,
}

impl StaticObstacleMap {
    pub fn new(width: u32, height: u32, resolution: f32) -> Self {
        Self {
            map: map::generate_white_map(width, height, resolution),
        }
    }
}
*/

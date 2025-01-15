//SPDX-FileCopyrightText: Ryuichi Ueda <ryuichiueda@gmail.com>
//SPDX-License-Identifier: BSD-3-Clause

use crate::map;
use nav_msgs::msg::OccupancyGrid;

#[derive(Default, Debug)]
pub struct Estimator {
    buffer: Vec<OccupancyGrid>,
}

impl Estimator {
    pub fn generate(&mut self, raw_buffer: &Vec<OccupancyGrid>, static_map: &OccupancyGrid) -> Option<OccupancyGrid> {
        const LIMIT_SEC: f64 = 0.9;
        const MIN_INTERVAL: f64 = 0.15;

        let mut now = raw_buffer.last()?.clone();

        if self.buffer.is_empty() {
            Self::subtract_static(&mut now, static_map);
            self.buffer.push(now);
            return None;
        }

        let prev = self.buffer.last()?.clone();
        let diff = map::time_diff(&prev.info.map_load_time, &now.info.map_load_time);
        if diff < MIN_INTERVAL {
            return None;
        }

        Self::subtract_static(&mut now, static_map);
        self.buffer.retain(|b| map::time_diff(&b.info.map_load_time, &now.info.map_load_time) < LIMIT_SEC );
        self.buffer.push(now);

        self.calculation()
    }
    
    fn subtract_static(dynamic_map: &mut OccupancyGrid, static_map: &OccupancyGrid) {
        dynamic_map.data.iter_mut().zip(static_map.data.iter())
            .for_each(|(d, s)| if *d < *s { *d = 0; }else { *d -= *s; });
    }

    pub fn calculation(&mut self) -> Option<OccupancyGrid> {
        let start = &self.buffer[0];

        Some(start.clone())
    }
}

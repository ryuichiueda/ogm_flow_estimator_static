//SPDX-FileCopyrightText: Ryuichi Ueda <ryuichiueda@gmail.com>
//SPDX-License-Identifier: BSD-3-Clause

use crate::map;
use nav_msgs::msg::OccupancyGrid;
use rand;
use rand::Rng;
use rand::rngs::ThreadRng;
use super::Error;

#[derive(Default, Debug)]
pub struct Trajectory {
    pub indexes: Vec<usize>,
}

impl Trajectory {
    pub fn add(&mut self, next_map: &OccupancyGrid, range: i32,
               rng: &mut ThreadRng) -> Result<(), Error> {
        let mut ans = vec![];
        let last_index = self.indexes.last().ok_or(Error::TrajectoryInit)?;
        let (cx, cy) = map::index_to_ixiy(*last_index, next_map.info.width, next_map.info.height)
                     .ok_or(Error::OutOfMap)?;

        for ix in (cx - range)..(cx + range + 1) {
            for iy in (cy - range)..(cy + range + 1) {
                if let Some(index) = map::ixiy_to_index(ix, iy, next_map.info.width, next_map.info.height) {
                    if next_map.data[index] > 0 {
                        ans.push(index);
                    }
                }
            }
        }

        if ans.is_empty() {
            return Err(Error::NotFound);
        }

        let selected = ans[rng.gen::<usize>()%ans.len()];
        self.indexes.push(selected);

        Ok(())
    }

    /*
    pub fn get_diff(&self, width: u32, height: u32, resolution: f32, rng: &mut ThreadRng) -> Option<(f64, f64)> {
        let s = self.get_start_pos(width, height, resolution, rng).unwrap();
        let e = self.get_end_pos(width, height, resolution, rng).unwrap();
        Some((e.0-s.0, e.1-s.1)) 
    }*/

    pub fn get_start_pos(&self, width: u32, height: u32, resolution: f32, rng: &mut ThreadRng) -> Option<(f64, f64)> {
        let s = map::index_to_real_pos(self.indexes[0], width, height, resolution)?;
        let x = s.0 as f64 + resolution as f64 * rng.gen::<f64>();
        let y = s.1 as f64 + resolution as f64 * rng.gen::<f64>();
        Some((x, y))
    }

    pub fn get_end_pos(&self, width: u32, height: u32, resolution: f32, rng: &mut ThreadRng) -> Option<(f64, f64)> {
        let s = map::index_to_real_pos(self.indexes[self.indexes.len()-1], width, height, resolution)?;
        let x = s.0 as f64 + resolution as f64 * rng.gen::<f64>();
        let y = s.1 as f64 + resolution as f64 * rng.gen::<f64>();
        Some((x, y))
    }
}

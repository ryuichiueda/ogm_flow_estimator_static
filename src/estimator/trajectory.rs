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
}

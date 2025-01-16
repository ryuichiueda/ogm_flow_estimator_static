//SPDX-FileCopyrightText: Ryuichi Ueda <ryuichiueda@gmail.com>
//SPDX-License-Identifier: BSD-3-Clause

use crate::map;
use nav_msgs::msg::OccupancyGrid;
use rand;
use rand::Rng;
use rand::seq::SliceRandom;
use rand::rngs::ThreadRng;

#[derive(Default, Debug)]
pub struct Estimator {
    buffer: Vec<OccupancyGrid>,
    trajectories: Vec<Trajectory>,
}

#[derive(Default, Debug)]
pub struct Trajectory {
    indexes: Vec<usize>,
}

impl Trajectory {
    pub fn add(&mut self, next_map: &OccupancyGrid, range: i32, rng: &mut ThreadRng) -> Result<(), Error> {
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

#[derive(Debug)]
pub enum Error {
    NoBuffer,
    TrajectoryInit,
    OutOfMap,
    NotFound,
}

impl Estimator {
    pub fn generate(&mut self, raw_buffer: &Vec<OccupancyGrid>, static_map: &OccupancyGrid) -> Result<Option<OccupancyGrid>, Error> {
        const LIMIT_SEC: f64 = 0.9;
        const MIN_INTERVAL: f64 = 0.15;

        let mut now = raw_buffer.last().ok_or(Error::NoBuffer)?.clone();
        if self.buffer.is_empty() {
            Self::subtract_static(&mut now, static_map);
            self.buffer.push(now);
            return Ok(None);
        }

        let prev = self.buffer.last().unwrap().clone();
        let diff = map::time_diff(&prev.info.map_load_time, &now.info.map_load_time);
        if diff < MIN_INTERVAL {
            return Ok(None);
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

    fn sampling(&mut self, num: usize) -> Vec<usize> {
        let map = &self.buffer[0];
        let sum: f64 = map.data.iter().map(|d| *d as usize).sum::<usize>() as f64;
        let step = sum / num as f64;

        let mut ans = vec![];

        let mut i_cell = 0;
        let mut rng = rand::thread_rng();
        let mut head: f64 = rng.gen::<f64>() * step;
        let mut accum: usize = 0;

        while head < sum && i_cell < map.data.len() {
            accum += map.data[i_cell] as usize;
            while head < accum as f64 {
                ans.push(i_cell);
                head += step;
            }

            i_cell += 1;
        }

        ans
    }

    fn pick_next_indexes(&self, cell_index: usize) -> Vec<usize> {
        vec![]
    }

    fn update_trajectory(&mut self, map_index: usize) -> Result<(), Error> {
        const RESOLUTION: f64 = 0.1;  // TODO: unify
        const MAX_SPEED: f64 = 2.5;

        let map = &self.buffer[map_index];
        let prev_map = &self.buffer[map_index-1];
        let diff = map::time_diff(&prev_map.info.map_load_time, &map.info.map_load_time);

        let max_traveling_cells = (MAX_SPEED * diff / RESOLUTION).ceil() as i32;

        dbg!("{:?}", &max_traveling_cells);
        let mut rng = rand::thread_rng();

        for traj in self.trajectories.iter_mut() {
            traj.add(&map, max_traveling_cells, &mut rng);
        }
        Ok(())
    }

    fn calculation(&mut self) -> Result<Option<OccupancyGrid>, Error> {
        self.trajectories = self.sampling(100).iter()
            .map(|s| Trajectory { indexes: vec![*s]}).collect();

        self.update_trajectory(1)?;

        let mut ans = self.buffer[0].clone();
        /*
        ans.data.iter_mut().for_each(|d| *d = 0 );
        for s in sample {
            ans.data[s] += 1;
        }*/
        Ok(Some(ans))
    }
}

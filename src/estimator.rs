//SPDX-FileCopyrightText: Ryuichi Ueda <ryuichiueda@gmail.com>
//SPDX-License-Identifier: BSD-3-Clause

use builtin_interfaces::msg::Duration;
use crate::map;
use std_msgs::msg::ColorRGBA;
use nav_msgs::msg::OccupancyGrid;
use geometry_msgs::msg::{Point, Vector3};
use rand;
use rand::Rng;
use rand::rngs::ThreadRng;
use chrono::Local;
use visualization_msgs::msg::Marker;
use visualization_msgs::msg::MarkerArray;

#[derive(Default, Debug)]
pub struct Estimator {
    buffer: Vec<OccupancyGrid>,
    trajectories: Vec<Trajectory>,
    marker_template: Marker,
}

#[derive(Default, Debug)]
pub struct Trajectory {
    indexes: Vec<usize>,
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

#[derive(Debug)]
pub enum Error {
    NoBuffer,
    TrajectoryInit,
    OutOfMap,
    NotFound,
}

impl Estimator {
    pub fn new() -> Self {
        let mut marker = Marker{
            type_:  0,
            ns:  "estimation".to_string(),
            scale: Vector3 { x: 0.02, y: 0.05, z: 0.1 },
            color: ColorRGBA{ r: 0.0, g: 0.0, b: 1.0, a: 1.0 },
            lifetime:  Duration{ sec: 1, nanosec: 0 },
            frame_locked:  true,
            ..Default::default()
        };

        let mut estimator = Self::default();
        estimator.marker_template = marker;

        estimator
    }

    pub fn generate(&mut self, raw_buffer: &Vec<OccupancyGrid>, static_map: &OccupancyGrid) -> Result<Option<MarkerArray>, Error> {
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

    fn update_trajectory(&mut self, map_index: usize) -> Result<(), Error> {
        const RESOLUTION: f64 = 0.1;  // TODO: unify
        const MAX_SPEED: f64 = 2.5;

        let map = &self.buffer[map_index];
        let prev_map = &self.buffer[map_index-1];
        let diff = map::time_diff(&prev_map.info.map_load_time, &map.info.map_load_time);

        let max_traveling_cells = (MAX_SPEED * diff / RESOLUTION).ceil() as i32;

        let mut rng = rand::thread_rng();

        for traj in self.trajectories.iter_mut() {
            let _ = traj.add(&map, max_traveling_cells, &mut rng);
        }
        Ok(())
    }

    fn forecast(&mut self) -> Result<Option<MarkerArray>, Error> {
        let mut ans = MarkerArray::default();
        self.marker_template.header = self.buffer[self.buffer.len()-1].header.clone();

        let mut rng = rand::thread_rng();
        let width = self.buffer[0].info.width;
        let height = self.buffer[0].info.height;
        let resolution = self.buffer[0].info.resolution;

        let start_time = map::time(&self.buffer[0]);
        let end_time = map::time(&self.buffer[self.buffer.len()-1]);
        let dt = end_time - start_time;

        for traj in &self.trajectories {
            let start = traj.indexes[0];
            let end = traj.indexes.last().unwrap();

            let (mut sx_real, mut sy_real) = match map::index_to_real_pos(start, width, height, resolution) {
                Some(pos) => pos,
                None => continue,
            };
            let (mut ex_real, mut ey_real) = match map::index_to_real_pos(*end, width, height, resolution) {
                Some(pos) => pos,
                None => continue,
            };

            sx_real += resolution as f64 * (((rng.gen::<usize>()%100) as f64) / 100.0);
            sy_real += resolution as f64 * (((rng.gen::<usize>()%100) as f64) / 100.0);
            ex_real += resolution as f64 * (((rng.gen::<usize>()%100) as f64) / 100.0);
            ey_real += resolution as f64 * (((rng.gen::<usize>()%100) as f64) / 100.0);

            let x_dist = (ex_real - sx_real) / dt;
            let y_dist = (ey_real - sy_real) / dt;

            self.marker_template.points.push( Point{ x: ex_real, y: ey_real, z: 0.01 } );
            self.marker_template.points.push( Point{ x: ex_real + x_dist , y: ey_real + y_dist, z: 0.01 } );
            self.marker_template.id = ans.markers.len() as i32;
            ans.markers.push(self.marker_template.clone());
            self.marker_template.points.clear();

            /*
            let mut t = from;
            while t < to {
                let sx = sx as f64 + ((rng.gen::<usize>()%100) as f64) / 100.0;
                let sy = sy as f64 + ((rng.gen::<usize>()%100) as f64) / 100.0;
                let ex = ex as f64 + ((rng.gen::<usize>()%100) as f64) / 100.0;
                let ey = ey as f64 + ((rng.gen::<usize>()%100) as f64) / 100.0;

                let x_dist = (ex - sx) * t / dt;
                let y_dist = (ey - sy) * t / dt;

                let forecast_x = (ex as f64 + x_dist).floor() as i32;
                let forecast_y = (ey as f64 + y_dist).floor() as i32;

                if let Some(index) = map::ixiy_to_index(forecast_x, forecast_y, width, height) {
                    ans.data[index] += 1;
                }

                t += delta;
            }*/
        }

        Ok(Some(ans))
    }

    fn calculation(&mut self) -> Result<Option<MarkerArray>, Error> {
        dbg!("START {:?}", Local::now());
//        let tm = &self.buffer[0].info.map_load_time;

        self.trajectories = self.sampling(100).iter()
            .map(|s| Trajectory { indexes: vec![*s]}).collect();

        for i in 1..self.buffer.len() {
            self.update_trajectory(i)?;
            self.trajectories.retain(|t| t.indexes.len() == i+1);
        }

        let ans = self.forecast();
        dbg!("END {:?}", Local::now());
        ans
    }
}

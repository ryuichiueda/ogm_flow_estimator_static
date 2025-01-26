//SPDX-FileCopyrightText: Ryuichi Ueda <ryuichiueda@gmail.com>
//SPDX-License-Identifier: BSD-3-Clause

mod trajectory;

use self::trajectory::Trajectory;
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

#[derive(Debug)]
pub enum Error {
    NoBuffer,
    TrajectoryInit,
    OutOfMap,
    NotFound,
}

#[derive(Default, Debug)]
pub struct Estimator {
    buffer: Vec<OccupancyGrid>,
    trajectories: Vec<Trajectory>,
    marker_template: Marker,
    rng: ThreadRng,
}

impl Estimator {
    pub fn new() -> Self {
        let marker = Marker{
            type_: 0,
            ns: "estimation".to_string(),
            scale: Vector3 { x: 0.02, y: 0.05, z: 0.1 },
            color: ColorRGBA{ r: 0.0, g: 0.0, b: 1.0, a: 1.0 },
            lifetime:  Duration{ sec: 1, nanosec: 0 },
            ..Default::default()
        };

        let mut estimator = Estimator {
            rng: rand::thread_rng(),
            ..Self::default()
        };

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
        //let mut rng = rand::thread_rng();
        let mut head: f64 = self.rng.gen::<f64>() * step;
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

        //let mut rng = rand::thread_rng();

        for traj in self.trajectories.iter_mut() {
            let _ = traj.add(&map, max_traveling_cells, &mut self.rng);
        }
        Ok(())
    }

    fn cross_check(ps: &mut Vec<((f64, f64), (f64, f64))>) -> usize {
        if ps.len() < 2 {
            return 0;
        }

        let iter_num = ps.len() - 1;
        let mut counter = 0;

        for i in 0..iter_num {
            let ax = ps[i].0.0;
            let ay = ps[i].0.1;
            let bx = ps[i].1.0;
            let by = ps[i].1.1;

            let cx = ps[i+1].0.0;
            let cy = ps[i+1].0.1;
            let dx = ps[i+1].1.0;
            let dy = ps[i+1].1.1;

            let ab_ac = (bx-ax)*(cy-ay)-(cx-ax)*(by-ay);
            let ab_ad = (bx-ax)*(dy-ay)-(dx-ax)*(by-ay);

            if ab_ac*ab_ad >= 0.0 { 
                    continue;
            }

            let cd_ca = (dx-cx)*(ay-cy)-(ax-cx)*(dy-cy);
            let cd_cb = (dx-cx)*(by-cy)-(bx-cx)*(dy-cy);

            if cd_ca*cd_cb >= 0.0 { 
                    continue;
            }

            ps[i].1.0 = dx;
            ps[i].1.1 = dy;
            ps[i+1].1.0 = bx;
            ps[i+1].1.1 = by;

            counter += 1;
        }

        counter
    }

    fn forecast(&mut self) -> Result<Option<MarkerArray>, Error> {
        let mut ans = MarkerArray::default();
        self.marker_template.header = self.buffer[self.buffer.len()-1].header.clone();

        let width = self.buffer[0].info.width;
        let height = self.buffer[0].info.height;
        let resolution = self.buffer[0].info.resolution;

        let start_time = map::time(&self.buffer[0]);
        let end_time = map::time(&self.buffer[self.buffer.len()-1]);
        let dt = end_time - start_time;

        let mut vectors = vec![];

        for traj in &self.trajectories {
            let e = traj.get_end_pos(width, height, resolution, &mut self.rng).unwrap();
            let s = traj.get_start_pos(width, height, resolution, &mut self.rng).unwrap();
            vectors.push( (s, e) );
        }

        for _ in 0..10 {
            let num = Self::cross_check(&mut vectors);
            dbg!("{:?}", &num);
            if num == 0 {
                break;
            }
        }

        for (s, e) in vectors {
            let x_dist = (e.0 - s.0) / dt;
            let y_dist = (e.1 - s.1) / dt;

            let ps = Point{ x: e.0, y: e.1, z: 0.01 };
            let pe = Point{ x: e.0 + x_dist , y: e.1 + y_dist, z: 0.01 };

            self.marker_template.points.push( ps );
            self.marker_template.points.push( pe );
            self.marker_template.id = ans.markers.len() as i32;
            ans.markers.push(self.marker_template.clone());
            self.marker_template.points.clear();
        }

        Ok(Some(ans))
    }

    fn calculation(&mut self) -> Result<Option<MarkerArray>, Error> {
        dbg!("START {:?}", Local::now());

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

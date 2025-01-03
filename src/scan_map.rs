//SPDX-FileCopyrightText: Ryuichi Ueda <ryuichiueda@gmail.com>
//SPDX-License-Identifier: BSD-3-Clause

use crate::map;
use nav_msgs::msg::OccupancyGrid;
use sensor_msgs::msg::LaserScan;

pub fn generate(width: u32, height: u32, resolution: f32, scan: &LaserScan) -> OccupancyGrid {
        let mut map = map::generate(width, height, resolution);
        scan_to_occupancy(&mut map, scan);

        map.header = scan.header.clone();
        map.info.map_load_time = map.header.stamp.clone();
        map
}

fn scan_to_occupancy(map: &mut OccupancyGrid, scan: &LaserScan) {
    for (i, range) in scan.ranges.iter().enumerate() {
        if range < &scan.range_min || range > &scan.range_max  {
            continue;
        }

        let angle = (&scan.angle_min + &scan.angle_increment*(i as f32)) as f64;
        let x = (*range as f64)*angle.cos();
        let y = (*range as f64)*angle.sin();

        plot(map, x, y, 100);
    }
}

fn plot(map: &mut OccupancyGrid, x: f64, y: f64, val: i8) {
    let res = map.info.resolution as f64;
    let map_x = ((x - map.info.origin.position.x)/res).floor() as i32;
    let map_y = ((y - map.info.origin.position.y)/res).floor() as i32;

    if map_x < 0 || map_y < 0 {
        return;
    }

    let index = (map_y as usize)*(map.info.width as usize)
                + (map_x as usize);

    map.data[index] = val;
}

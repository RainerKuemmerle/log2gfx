extern crate nalgebra as na;

use core::f32;

use crate::datastream::robot_data::RobotLaser;

use super::gridmap;

fn bresenham(x1: i32, y1: i32, x2: i32, y2: i32) -> Vec<[i32; 2]> {
    fn bresenham_core(x1: i32, y1: i32, x2: i32, y2: i32) -> Vec<[i32; 2]> {
        let mut points = vec![];
        let dx = x2 - x1;
        let dy = y2 - y1;
        let mut x = x1;
        let mut y = y1;
        let mut error = dx / 2;
        while x <= x2 {
            points.push([x, y]);
            x += 1;
            error -= dy;
            if error < 0 {
                y += 1;
                error += dx;
            }
        }
        return points;
    }
    if x2 < x1 {
        let mut reversed = bresenham_core(x2, y2, x1, y1);
        reversed.reverse();
        return reversed;
    }
    if (y2 - y1).abs() > (x2 - x1).abs() {
        let mut swapped = bresenham_core(y1, x1, y2, x2);
        for point in swapped.iter_mut() {
            point.swap(0, 1);
        }
        return swapped;
    }

    return bresenham_core(x1, y1, x2, y1);
}

#[derive(Debug, Copy, Clone)]
struct FrequencyMapCell {
    hits: i32,
    misses: i32,
}

pub struct FrequencyMap {
    map: gridmap::GridMap<FrequencyMapCell>,
}

impl FrequencyMap {
    pub fn new(size: na::Vector2<usize>, resolution: f64, offset: na::Vector2<f64>) -> Self {
        let default_cell = FrequencyMapCell { hits: 0, misses: 0 };
        let map = gridmap::GridMap::new(size, resolution, offset, default_cell);
        Self { map }
    }

    pub fn integrate_scan(
        &mut self,
        laser: &RobotLaser,
        robot_pose: na::Isometry2<f64>,
        max_range: Option<f64>,
        max_usable_range: Option<f64>,
        gain: Option<i32>,
    ) {
        let my_max_range = laser
            .laser_params
            .max_range
            .min(max_range.unwrap_or(f64::INFINITY)) as f32;
        let my_usable_range = max_usable_range.unwrap_or(my_max_range.into()) as f32;

        let laser_pose = robot_pose * laser.laser_params.laser_pose;
        let start = self.map.world2map(&laser_pose.translation.vector);
        for (i, range) in laser.ranges.iter().enumerate() {
            if *range > my_max_range {
                continue;
            }
            let mut r = *range;
            let mut cropped = false;
            if r > my_usable_range {
                r = my_usable_range;
                cropped = true;
            }
            let beam = na::Vector2::new(r as f64, 0.);
            let beam_end_point = laser_pose * laser.laser_params.beam_isometry(i) * beam;
            let end = self.map.world2map(&beam_end_point);

            let line = bresenham(start.x, end.x, start.y, end.y);
            for point in line.iter() {
                match self.map.cell_scalar(point[0], point[1]) {
                    None => continue,
                    Some(c) => c.misses += gain.unwrap_or(1),
                }
            }
            if !self.map.is_inside_scalar(end.x, end.y) {
                continue;
            }
            if !cropped {
                match self.map.cell_scalar(end[0], end[1]) {
                    None => continue,
                    Some(c) => c.hits += gain.unwrap_or(1),
                }
            }
        }
    }
}

extern crate nalgebra as na;

use core::f32;
use std::iter::zip;

use crate::datastream::robot_data::RobotLaser;

use super::{floatmap::FloatMap, gridmap};

fn bresenham(x1: i32, y1: i32, x2: i32, y2: i32) -> Vec<[i32; 2]> {
    let mut coordinates = vec![];
    let dx: i32 = i32::abs(x2 - x1);
    let dy: i32 = i32::abs(y2 - y1);
    let sx: i32 = if x1 < x2 { 1 } else { -1 };
    let sy: i32 = if y1 < y2 { 1 } else { -1 };

    let mut error: i32 = (if dx > dy { dx } else { -dy }) / 2;
    let mut current_x: i32 = x1;
    let mut current_y: i32 = y1;
    loop {
        coordinates.push([current_x, current_y]);

        if current_x == x2 && current_y == y2 {
            break;
        }

        let error2: i32 = error;

        if error2 > -dx {
            error -= dy;
            current_x += sx;
        }
        if error2 < dy {
            error += dx;
            current_y += sy;
        }
    }
    coordinates
}

#[derive(Debug, Copy, Clone)]
pub struct FrequencyMapCell {
    hits: i32,
    misses: i32,
}

pub struct FrequencyMap {
    pub map: gridmap::GridMap<FrequencyMapCell>,
}

impl FrequencyMap {
    pub fn new(size: [usize; 2], resolution: f64, offset: na::Vector2<f64>) -> Self {
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
            let beam = na::Point2::new(r as f64, 0.);
            let beam_end_point = laser_pose * laser.laser_params.beam_isometry(i) * beam;
            let end = self.map.world2map(&beam_end_point.coords);

            let line = bresenham(start.x, start.y, end.x, end.y);
            for point in line.iter() {
                match self.map.cell_mut(point[0], point[1]) {
                    Some(c) => c.misses += gain.unwrap_or(1),
                    None => continue,
                }
            }
            if !self.map.is_inside(end.x, end.y) {
                continue;
            }
            if !cropped {
                match self.map.cell_mut(end[0], end[1]) {
                    Some(c) => c.hits += gain.unwrap_or(1),
                    None => continue,
                }
            }
        }
    }

    pub fn compute_occupancy_map(&self) -> FloatMap {
        let default_cell = -1.0f32;
        let mut map = gridmap::GridMap::new(
            self.map.size,
            self.map.resolution,
            self.map.offset,
            default_cell,
        );

        for (hits_misses, occupancy) in zip(self.map.cells(), map.cells_mut()) {
            if hits_misses.misses > 0 {
                *occupancy = hits_misses.hits as f32 / hits_misses.misses as f32;
            } else {
                *occupancy = default_cell;
            }
        }

        FloatMap { map }
    }
}

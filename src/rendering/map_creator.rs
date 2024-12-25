extern crate nalgebra as na;

use std::io::Write;

use crate::datastream::robot_data::RobotLaser;

use super::boundaries::boundaries;
use super::frequencymap::FrequencyMap;
use super::map_creator_parameter::MapCreatorParameter;

pub struct MapCreator {
    pub parameter: MapCreatorParameter,
    pub boundaries_min: na::Vector2<f64>,
    pub boundaries_max: na::Vector2<f64>,
    pub fmap: Option<FrequencyMap>,
}

impl MapCreator {
    pub fn new(parameter: MapCreatorParameter) -> Self {
        Self {
            parameter,
            boundaries_min: na::Vector2::new(f64::INFINITY, f64::INFINITY),
            boundaries_max: na::Vector2::new(f64::NEG_INFINITY, f64::NEG_INFINITY),
            fmap: None,
        }
    }

    pub fn update_boundaries(&mut self, scans: &[RobotLaser]) {
        for rl in scans.iter() {
            if self.parameter.zero_first_pose {
                self.parameter.zero_first_pose = false;
                self.parameter.offset = rl.odom_pose.inverse();
            }
            let my_max_range = self.parameter.max_range.min(rl.laser_params.max_range);
            let my_usable_range = self
                .parameter
                .max_usable_range
                .min(rl.laser_params.max_range);
            boundaries(
                &mut self.boundaries_min,
                &mut self.boundaries_max,
                &self.parameter.offset,
                rl,
                Some(my_max_range as f32),
                Some(my_usable_range as f32),
            );
        }
    }

    pub fn integrate_scans(&mut self, scans: &[RobotLaser]) {
        if self.fmap.is_none() {
            panic!("Called integrate_scans without an allocated map");
        }

        let map = self.fmap.as_mut().unwrap();

        if self.parameter.verbose {
            print!("Integrating scans ... ");
            let _ = std::io::stdout().flush();
        }
        for rl in scans.iter() {
            let my_max_range = self.parameter.max_range.min(rl.laser_params.max_range);
            let my_usable_range = self
                .parameter
                .max_usable_range
                .min(rl.laser_params.max_range);

            map.integrate_scan(
                rl,
                self.parameter.offset * rl.odom_pose,
                Some(my_max_range),
                Some(my_usable_range),
                None,
            );
        }
        if self.parameter.verbose {
            println!("done.");
        }
    }

    pub fn allocate_map(&mut self) {
        if self.parameter.verbose {
            println!(
                "Boundaries: {:.3} {:.3} -> {:.3} {:.3}",
                self.boundaries_min.x,
                self.boundaries_min.y,
                self.boundaries_max.x,
                self.boundaries_max.y
            );
        }
        let border = na::Vector2::new(self.parameter.border, self.parameter.border);
        let boundaries_min = self.boundaries_min - border;
        let boundaries_max = self.boundaries_max + border;
        if self.parameter.verbose {
            println!(
                "Extended Boundaries: {:.3} {:.3} -> {:.3} {:.3}",
                boundaries_min.x, boundaries_min.y, boundaries_max.x, boundaries_max.y
            );
        }

        let dsize = boundaries_max - boundaries_min;
        let isize = (dsize / self.parameter.resolution)
            .try_cast::<usize>()
            .unwrap();

        if self.parameter.verbose {
            println!("Allocating map size {} x {}", isize.x, isize.y)
        }
        self.fmap = Some(FrequencyMap::new(
            [isize.x, isize.y],
            self.parameter.resolution,
            boundaries_min,
        ));
    }
}

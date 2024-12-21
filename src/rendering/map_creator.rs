extern crate nalgebra as na;

use core::f64;

use crate::datastream::robot_data::RobotLaser;

use super::boundaries::boundaries;
use super::frequencymap::{self, FrequencyMap};
use super::map_creator_parameter::{self, MapCreatorParameter};

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

    pub fn update_boundaries(&mut self, scans: &Vec<RobotLaser>) {
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
                &rl,
                Some(my_max_range as f32),
                Some(my_usable_range as f32),
            );
        }
    }
}

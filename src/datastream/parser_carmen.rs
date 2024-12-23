extern crate nalgebra as na;

use std::fs::File;
use std::io::{self, BufRead};
use std::path::{Path, PathBuf};

use super::parser;
use super::robot_data;

pub struct CarmenFile {
    pub filename: PathBuf,
}

fn read_lines<P>(filename: P) -> io::Result<io::Lines<io::BufReader<File>>>
where
    P: AsRef<Path>,
{
    let file = File::open(filename)?;
    Ok(io::BufReader::new(file).lines())
}

fn read_robotlaser(line: &str) -> Option<robot_data::RobotLaser> {
    if !line.starts_with("ROBOTLASER") {
        return None;
    }
    let mut tokens = line.split_whitespace();
    let _tag = tokens.next().unwrap();
    let _laser_type: i32 = tokens.next().unwrap().parse().unwrap();
    let angle: f64 = tokens.next().unwrap().parse().unwrap();
    let _fov: f64 = tokens.next().unwrap().parse().unwrap();
    let angular_step: f64 = tokens.next().unwrap().parse().unwrap();
    let max_range: f64 = tokens.next().unwrap().parse().unwrap();
    let _accuracy: f64 = tokens.next().unwrap().parse().unwrap();
    let _remission_mode: i32 = tokens.next().unwrap().parse().unwrap();

    // parsing the beams
    let num_beams: i32 = tokens.next().unwrap().parse().unwrap();
    let mut ranges = Vec::with_capacity(num_beams as usize);
    for _ in 0..num_beams {
        let range: f32 = tokens.next().unwrap().parse().unwrap();
        ranges.push(range);
    }

    let num_remissions: i32 = tokens.next().unwrap().parse().unwrap();
    // TODO(Rainer): Use advance_by later
    for _ in 0..num_remissions {
        tokens.next();
    }

    let laser_x: f64 = tokens.next().unwrap().parse().unwrap();
    let laser_y: f64 = tokens.next().unwrap().parse().unwrap();
    let laser_theta: f64 = tokens.next().unwrap().parse().unwrap();
    let laser_pose_global = na::Isometry2::new(na::Vector2::new(laser_x, laser_y), laser_theta);

    let robot_x: f64 = tokens.next().unwrap().parse().unwrap();
    let robot_y: f64 = tokens.next().unwrap().parse().unwrap();
    let robot_theta: f64 = tokens.next().unwrap().parse().unwrap();
    let robot_pose_global = na::Isometry2::new(na::Vector2::new(robot_x, robot_y), robot_theta);

    // Relative laser pose and the parameters finally
    let laser_pose_relative = robot_pose_global.inverse() * laser_pose_global;
    let laser_params =
        robot_data::LaserParameters::new(laser_pose_relative, angle, angular_step, max_range);

    Some(robot_data::RobotLaser::new(
        laser_params,
        robot_pose_global,
        ranges,
    ))
}

impl parser::Parser for CarmenFile {
    fn parse(&self) -> Vec<robot_data::RobotLaser> {
        match read_lines(&self.filename) {
            Ok(lines) => lines
                .map_while(Result::ok)
                .filter_map(|l| read_robotlaser(&l))
                .collect(),
            Err(_) => Vec::new(),
        }
    }
}

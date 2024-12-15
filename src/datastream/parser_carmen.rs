extern crate nalgebra as na;

use std::fs::File;
use std::io::{self, BufRead};
use std::path::Path;

use super::parser;
use super::robot_data;

pub struct CarmenFile {
    pub filename: String,
}

fn read_lines<P>(filename: P) -> io::Result<io::Lines<io::BufReader<File>>>
where
    P: AsRef<Path>,
{
    let file = File::open(filename)?;
    Ok(io::BufReader::new(file).lines())
}

fn read_robotlaser(line: &String) -> robot_data::RobotLaser {
    let mut tokens = line.split_whitespace();
    let tag = tokens.next().unwrap();
    let laser_type: i32 = tokens.next().unwrap().parse().unwrap();
    let angle: f64 = tokens.next().unwrap().parse().unwrap();
    let _fov: f64 = tokens.next().unwrap().parse().unwrap();
    let angular_step: f64 = tokens.next().unwrap().parse().unwrap();
    let max_range: f64 = tokens.next().unwrap().parse().unwrap();
    let accuracy: f64 = tokens.next().unwrap().parse().unwrap();
    let remission_mode: i32 = tokens.next().unwrap().parse().unwrap();

    // parsing the beams
    let num_beams: i32 = tokens.next().unwrap().parse().unwrap();
    let mut ranges = Vec::new();
    ranges.reserve(num_beams as usize);
    for _ in 0..num_beams {
        let range: f32 = tokens.next().unwrap().parse().unwrap();
        ranges.push(range);
    }

    let num_remissions: i32 = tokens.next().unwrap().parse().unwrap();
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
    let laser_params = robot_data::LaserParameters::new(
        laser_pose_relative,
        laser_type,
        num_beams,
        angle,
        angular_step,
        max_range,
        accuracy,
        remission_mode,
    );

    for _ in 0..5 {
        tokens.next();
    }
    let timestamp: f64 = tokens.next().unwrap().parse().unwrap();
    let hostname = tokens.next().unwrap();
    let logger_timestamp: f64 = tokens.next().unwrap().parse().unwrap();

    let data_packet = robot_data::DataPacket::new(
        String::from(tag),
        robot_data::Type::RobotLaser,
        timestamp,
        logger_timestamp,
        String::from(hostname),
    );

    robot_data::RobotLaser::new(data_packet, laser_params, robot_pose_global, ranges)
}

impl parser::Parser for CarmenFile {
    fn parse(&self) -> Vec<robot_data::RobotLaser> {
        let mut laser_readings = Vec::new();
        if let Ok(lines) = read_lines(&self.filename) {
            for line in lines.flatten() {
                if !line.starts_with("ROBOTLASER") {
                    continue;
                }
                laser_readings.push(read_robotlaser(&line));
            }
        }
        laser_readings
    }
}

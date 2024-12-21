extern crate nalgebra as na;
use crate::datastream::robot_data::RobotLaser;

pub fn boundaries(
    min: &mut na::Vector2<f64>,
    max: &mut na::Vector2<f64>,
    offset: &na::Isometry2<f64>,
    scan: &RobotLaser,
    max_range: Option<f32>,
    usable_range: Option<f32>,
) {
    let tp = offset * scan.laser_pose();

    // consider robot pose;
    let robot_pose = tp.translation;
    min.x = min.x.min(robot_pose.x);
    min.y = min.y.min(robot_pose.y);
    max.x = max.x.max(robot_pose.x);
    max.y = max.y.max(robot_pose.y);
    let max_range = (scan.laser_params.max_range as f32).min(max_range.unwrap_or(f32::INFINITY));
    for (i, range) in scan.ranges.iter().enumerate() {
        let mut r = *range;
        if r >= max_range {
            continue;
        }
        if usable_range.is_some() {
            r = r.min(usable_range.unwrap());
        }
        let point = na::Vector2::new(r as f64, 0.);
        let transformed_point = tp * scan.laser_params.beam_isometry(i) * point;
        min.x = min.x.min(transformed_point.x);
        min.y = min.y.min(transformed_point.y);
        max.x = max.x.max(transformed_point.x);
        max.y = max.y.max(transformed_point.y);
    }
}

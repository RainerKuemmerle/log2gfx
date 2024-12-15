extern crate nalgebra as na;
use crate::datastream::robot_data::RobotLaser;

pub fn boundaries(
    min: &mut na::Vector2<f64>,
    max: &mut na::Vector2<f64>,
    offset: &na::Isometry2<f64>,
    scan: &RobotLaser,
    usable_range: f32,
) {
    let tp = offset * scan.laser_pose();

    // consider robot pose;
    let ropot_pose = tp.translation;
    min.x = min.x.min(ropot_pose.x);
    min.y = min.y.min(ropot_pose.y);
    max.x = max.x.max(ropot_pose.x);
    max.y = max.y.max(ropot_pose.y);
    let max_range = scan.laser_params.max_range as f32;
    for i in 0..scan.ranges.len() {
        let mut r = scan.ranges[i];
        if r >= max_range {
            continue;
        }
        if usable_range > 0.0 && r > usable_range {
            r = usable_range;
        }
        let point = na::Vector2::new(r as f64, 0.);
        let transformed_point = tp * scan.laser_params.beam_isometry(i as i32) * point;
        min.x = min.x.min(transformed_point.x);
        min.y = min.y.min(transformed_point.y);
        max.x = max.x.max(transformed_point.x);
        max.y = max.y.max(transformed_point.y);
    }
}

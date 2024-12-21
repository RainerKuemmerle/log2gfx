extern crate nalgebra as na;
use crate::datastream::robot_data::RobotLaser;

fn update_min_max(
    point: &na::Vector2<f64>,
    min: &mut na::Vector2<f64>,
    max: &mut na::Vector2<f64>,
) {
    min.x = min.x.min(point.x);
    min.y = min.y.min(point.y);
    max.x = max.x.max(point.x);
    max.y = max.y.max(point.y);
}

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
    update_min_max(&tp.translation.vector, min, max);
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
        update_min_max(&transformed_point, min, max);
    }
}

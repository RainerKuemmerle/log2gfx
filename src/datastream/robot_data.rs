extern crate nalgebra as na;

pub struct LaserParameters {
    pub laser_pose: na::Isometry2<f64>,
    pub first_beam_theta: f64,
    pub angular_step: f64,
    pub max_range: f64,
}

impl LaserParameters {
    pub fn new(
        laser_pose: na::Isometry2<f64>,
        first_beam_theta: f64,
        angular_step: f64,
        max_range: f64,
    ) -> Self {
        Self {
            laser_pose,
            first_beam_theta,
            angular_step,
            max_range,
        }
    }

    pub fn beam_angle(&self, index: usize) -> f64 {
        self.first_beam_theta + index as f64 * self.angular_step
    }

    pub fn beam_isometry(&self, index: usize) -> na::Isometry2<f64> {
        let theta = self.beam_angle(index);
        na::Isometry2::new(na::Vector2::zeros(), theta)
    }
}

pub struct RobotLaser {
    pub laser_params: LaserParameters,
    pub odom_pose: na::Isometry2<f64>,
    pub ranges: Vec<f32>,
}
impl RobotLaser {
    pub fn new(
        laser_params: LaserParameters,
        odom_pose: na::Isometry2<f64>,
        ranges: Vec<f32>,
    ) -> Self {
        Self {
            laser_params,
            odom_pose,
            ranges,
        }
    }

    pub fn laser_pose(&self) -> na::Isometry2<f64> {
        self.odom_pose * self.laser_params.laser_pose
    }
}

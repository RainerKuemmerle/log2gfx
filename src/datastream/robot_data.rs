extern crate nalgebra as na;

pub enum Type {
    Unknown,
    RobotLaser,
}

pub struct DataPacket {
    pub tag: String,
    pub data_type: Type,
    pub timestamp: f64,
    pub logger_timestamp: f64,
    pub hostname: String,
}
impl DataPacket {
    pub fn new(
        tag: String,
        data_type: Type,
        timestamp: f64,
        logger_timestamp: f64,
        hostname: String,
    ) -> Self {
        Self {
            tag,
            data_type,
            timestamp,
            logger_timestamp,
            hostname,
        }
    }
}

pub struct LaserParameters {
    pub laser_pose: na::Isometry2<f64>,
    pub laser_type: i32,
    pub num_beams: i32,
    pub first_beam_theta: f64,
    pub angular_step: f64,
    pub max_range: f64,
    pub accuracy: f64,
    pub remission_mode: i32,
}

impl LaserParameters {
    pub fn new(
        laser_pose: na::Isometry2<f64>,
        laser_type: i32,
        num_beams: i32,
        first_beam_theta: f64,
        angular_step: f64,
        max_range: f64,
        accuracy: f64,
        remission_mode: i32,
    ) -> Self {
        Self {
            laser_pose,
            laser_type,
            num_beams,
            first_beam_theta,
            angular_step,
            max_range,
            accuracy,
            remission_mode,
        }
    }

    pub fn beam_angle(&self, index: i32) -> f64 {
        self.first_beam_theta + index as f64 * self.angular_step
    }

    pub fn beam_isometry(&self, index: i32) -> na::Isometry2<f64> {
        let theta = self.beam_angle(index);
        na::Isometry2::new(na::Vector2::zeros(), theta)
    }
}

pub struct RobotLaser {
    pub data_packet: DataPacket,
    pub laser_params: LaserParameters,
    pub odom_pose: na::Isometry2<f64>,
    pub ranges: Vec<f32>,
}
impl RobotLaser {
    pub fn new(
        data_packet: DataPacket,
        laser_params: LaserParameters,
        odom_pose: na::Isometry2<f64>,
        ranges: Vec<f32>,
    ) -> Self {
        Self {
            data_packet,
            laser_params,
            odom_pose,
            ranges,
        }
    }

    pub fn laser_pose(&self) -> na::Isometry2<f64> {
        self.odom_pose * self.laser_params.laser_pose
    }

    pub fn crop(&mut self, max_distance: f32, min_distance: f32) {
        for range in &mut self.ranges {
            *range = range.min(max_distance).max(min_distance);
        }
    }

    pub fn cartesian(&self) -> Vec<na::Vector2<f64>> {
        let mut coords = Vec::new();

        let max_range = self.laser_params.max_range as f32;
        // compute the cartesian coords
        for i in 0..self.ranges.len() {
            let range = self.ranges[i];
            if range >= max_range {
                continue;
            }
            let p = na::Vector2::new(range as f64, 0.0);
            coords.push(self.laser_params.beam_isometry(i as i32) * p);
        }
        coords
    }
}

extern crate nalgebra as na;

pub enum Type {
    Unknown,
    RobotLaser,
}

pub struct DataPacket {
    tag: String,
    data_type: Type,
    timestamp: f64,
    logger_timestamp: f64,
    hostname: String,
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
    laser_pose: na::Isometry2<f64>,
    laser_type: i32,
    num_beams: i32,
    first_beam_theta: f64,
    angular_step: f64,
    max_range: f64,
    accuracy: f64,
    remission_mode: i32,
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
}

pub struct RobotLaser {
    data_packet: DataPacket,
    laser_params: LaserParameters,
    odom_pose: na::Isometry2<f64>,
    ranges: Vec<f32>,
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
}

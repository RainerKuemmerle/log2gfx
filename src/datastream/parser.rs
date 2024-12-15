use super::robot_data;

pub trait Parser {
  fn parse(&self) -> Vec<robot_data::RobotLaser>;
}

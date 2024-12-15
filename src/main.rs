extern crate nalgebra as na;

mod datastream;
use datastream::{parser::Parser, parser_carmen::CarmenFile};
mod rendering;
use rendering::boundaries::boundaries;

fn main() {
    let carmen_file = CarmenFile {
        filename: String::from("/home/goki/workspace/data/2d/intel/intel.gm2dl"),
    };
    let data = carmen_file.parse();
    println!("Number of laser readings: {}", data.len());

    let offset = na::Isometry2::identity();
    let mut boundary_min = na::Vector2::new(1e200, 1e200);
    let mut boundary_max = -boundary_min;

    for scan in data {
        boundaries(&mut boundary_min, &mut boundary_max, &offset, &scan, -1.)
    }
    println!("Boundary min: {}", boundary_min);
    println!("Boundary max: {}", boundary_max);
}

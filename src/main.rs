extern crate nalgebra as na;

mod datastream;
use datastream::{parser::Parser, parser_carmen::CarmenFile};
mod rendering;
use rendering::map_creator::{self, MapCreator};
use rendering::map_creator_parameter::{self, MapCreatorParameter};

fn main() {
    let carmen_file = CarmenFile {
        filename: String::from("/home/goki/workspace/data/2d/intel/intel.gm2dl"),
    };
    let data = carmen_file.parse();
    println!("Number of laser readings: {}", data.len());

    let mut map_creator_parameter = MapCreatorParameter::default();
    map_creator_parameter.verbose = true;
    let mut map_creator = MapCreator::new(map_creator_parameter);

    map_creator.update_boundaries(&data);
    println!("Boundary min: {}", map_creator.boundaries_min);
    println!("Boundary max: {}", map_creator.boundaries_max);

    map_creator.allocate_map();
    map_creator.integrate_scans(&data);
}

mod datastream;
use datastream::{parser::Parser, parser_carmen::CarmenFile};

fn main() {
    let carmen_file = CarmenFile {
        filename: String::from("/home/goki/workspace/data/2d/intel/intel.gm2dl"),
    };
    let data = carmen_file.parse();
    println!("Number of laser readings: {}", data.len());
}

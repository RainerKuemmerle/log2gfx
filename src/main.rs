extern crate nalgebra as na;

use std::path::PathBuf;

use clap::Parser as ClapParser;

mod datastream;
use datastream::{parser::Parser, parser_carmen::CarmenFile};
mod rendering;
use rendering::map_creator::MapCreator;
use rendering::map_creator_parameter::MapCreatorParameter;

#[derive(ClapParser)]
#[command(version, about, long_about = None)]
struct Cli {
    /// Input logfile for rendering
    input: PathBuf,

    /// Output filename
    output: PathBuf,

    // Sets a custom config file
    // #[arg(short, long, value_name = "FILE")]
    // config: Option<PathBuf>,
    /// Turn verbose logging on
    #[arg(short, long)]
    verbose: bool,
}

fn main() {
    let cli = Cli::parse();

    let mut map_creator_parameter = MapCreatorParameter::default();
    map_creator_parameter.verbose = cli.verbose;

    let carmen_file = CarmenFile {
        filename: cli.input,
    };
    let data = carmen_file.parse();
    if cli.verbose {
        println!("Number of laser readings: {}", data.len());
    }

    let mut map_creator = MapCreator::new(map_creator_parameter);

    map_creator.update_boundaries(&data);
    map_creator.allocate_map();
    map_creator.integrate_scans(&data);

    let fmap = map_creator.fmap.as_ref().unwrap();
    let occupancy_map = fmap.compute_occupancy_map();

    let _result = occupancy_map.save_as_ppm(&cli.output);
}

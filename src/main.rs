use std::path::PathBuf;

use clap::Parser as ClapParser;

mod datastream;
use datastream::{parser::Parser, parser_carmen::CarmenFile};
mod rendering;
use rendering::map_creator::MapCreator;
use rendering::map_creator_parameter::MapCreatorParameter;
mod drawing;
use drawing::map_drawer::MapDrawer;

#[derive(ClapParser)]
#[command(version, about, long_about = None)]
struct Cli {
    /// Input logfile for rendering
    input: PathBuf,

    /// Output filename
    output: PathBuf,

    /// Turn verbose logging on
    #[arg(short, long)]
    verbose: bool,

    /// Draw the path of the robot
    #[arg(long)]
    draw_path: bool,
}

fn to_map_drawer(map_creator: MapCreator) -> MapDrawer {
    let fmap = map_creator.fmap.as_ref().unwrap();
    let width = fmap.map.size[0] as u32;
    let height = fmap.map.size[1] as u32;
    let img_data = fmap.compute_occupancy_map().to_pixels();
    let img = tiny_skia::Pixmap::from_vec(
        img_data,
        tiny_skia::IntSize::from_wh(width, height).unwrap(),
    );
    MapDrawer::new(
        map_creator.parameter.resolution,
        [fmap.map.offset.x, fmap.map.offset.y],
        img.unwrap(),
    )
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

    let mut map_drawer = {
        let mut map_creator = MapCreator::new(map_creator_parameter);

        map_creator.update_boundaries(&data);
        map_creator.allocate_map();
        map_creator.integrate_scans(&data);
        to_map_drawer(map_creator)
    };

    if cli.draw_path {
        if cli.verbose {
            print!("Drawing the path ... ");
        }
        map_drawer.draw_path(&data);
        if cli.verbose {
            println!("done.")
        }
    }

    let _result = map_drawer.to_image().save(cli.output);
}

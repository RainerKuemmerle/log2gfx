extern crate nalgebra as na;

use std::io::Write;
use std::iter::zip;
use std::path::PathBuf;

use clap::Parser as ClapParser;

mod datastream;
use datastream::robot_data::RobotLaser;
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

    /// Resolution of the image in meter per pixel
    #[arg(short, long, default_value_t = 0.1)]
    resolution: f64,

    /// Draw the path of the robot
    #[arg(long)]
    draw_path: bool,

    /// Highlight scans in the map
    #[arg(long, num_args = 1..)]
    scan: Vec<usize>,

    /// Offset for the map in [m, m, rad]
    #[arg(long, num_args = 3)]
    offset: Vec<f64>,
}

fn compute_length(scans: &[RobotLaser]) -> f64 {
    let mut len = 0.;
    for (curr, next) in zip(scans.iter(), scans.iter().skip(1)) {
        len += (next.odom_pose.translation.vector - curr.odom_pose.translation.vector).norm();
    }
    len
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
        map_creator.parameter,
        [fmap.map.offset.x, fmap.map.offset.y],
        img.unwrap(),
    )
}

fn main() {
    let cli = Cli::parse();

    let offset = if !cli.offset.is_empty() {
        na::Isometry2::new(
            na::Vector2::new(cli.offset[0], cli.offset[1]),
            cli.offset[2],
        )
    } else {
        na::Isometry2::identity()
    };

    let map_creator_parameter = MapCreatorParameter {
        verbose: cli.verbose,
        resolution: cli.resolution,
        offset,
        ..Default::default()
    };

    let carmen_file = CarmenFile {
        filename: cli.input,
    };
    let data = carmen_file.parse();
    if cli.verbose {
        println!("Number of laser readings: {}", data.len());
        println!("Trajectory length: {:.3} m", compute_length(&data));
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
            let _ = std::io::stdout().flush();
        }
        map_drawer.draw_path(&data);
        if cli.verbose {
            println!("done.")
        }
    }

    if !cli.scan.is_empty() {
        if cli.verbose {
            print!("Drawing scans ... ");
            let _ = std::io::stdout().flush();
        }
        for idx in cli.scan.iter().filter(|&x| *x < data.len()) {
            if cli.verbose {
                print!("{} ", idx);
                let _ = std::io::stdout().flush();
            }
            map_drawer.draw_scan(&data[*idx], Some(map_creator_parameter.max_usable_range));
        }
        if cli.verbose {
            println!("done.")
        }
    }

    let _result = map_drawer.to_image().save(cli.output);
}

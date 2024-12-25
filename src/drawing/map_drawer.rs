extern crate nalgebra as na;

use crate::{
    datastream::robot_data::RobotLaser, rendering::map_creator_parameter::MapCreatorParameter,
};
use image::RgbaImage;

pub struct MapDrawer {
    pub parameter: MapCreatorParameter,
    pub offset: [f64; 2],
    pub img: tiny_skia::Pixmap,
}

impl MapDrawer {
    pub fn new(parameter: MapCreatorParameter, offset: [f64; 2], img: tiny_skia::Pixmap) -> Self {
        Self {
            parameter,
            offset,
            img,
        }
    }

    pub fn to_image(&self) -> RgbaImage {
        let w = self.img.width();
        let h = self.img.height();
        let mut rgba = RgbaImage::new(w, h);

        for y in 0..h {
            for x in 0..w {
                let s = self.img.pixel(x, y).unwrap().demultiply();
                let pixel = rgba.get_pixel_mut(x, y);
                *pixel = image::Rgba([s.red(), s.green(), s.blue(), s.alpha()]);
            }
        }
        rgba
    }

    fn world2map(&self, wp: [f64; 2]) -> [f32; 2] {
        let map_point = [
            (wp[0] - self.offset[0]) / self.parameter.resolution,
            self.img.height() as f64 - (wp[1] - self.offset[1]) / self.parameter.resolution,
        ];
        map_point.map(|c| c as f32)
    }

    pub fn draw_path(&mut self, scans: &[RobotLaser]) {
        if scans.is_empty() {
            return;
        }

        let mut paint = tiny_skia::Paint::default();
        paint.set_color_rgba8(231, 130, 132, 255);
        paint.anti_alias = true;

        let path = {
            let mut pb = tiny_skia::PathBuilder::new();
            let pose = self.parameter.offset * scans[0].odom_pose;
            let coords = self.world2map([pose.translation.x, pose.translation.y]);
            pb.move_to(coords[0], coords[1]);
            for s in scans.iter().skip(1) {
                let pose = self.parameter.offset * s.odom_pose;
                let coords = self.world2map([pose.translation.x, pose.translation.y]);
                pb.line_to(coords[0], coords[1]);
            }
            pb.finish().unwrap()
        };

        let stroke = tiny_skia::Stroke::default();
        // stroke.width = 2.0;
        // stroke.line_cap = tiny_skia::LineCap::Round;

        self.img.stroke_path(
            &path,
            &paint,
            &stroke,
            tiny_skia::Transform::identity(),
            None,
        );
    }

    pub fn draw_scan(&mut self, scan: &RobotLaser, max_usable_range: Option<f64>) {
        let usable_range = scan
            .laser_params
            .max_range
            .min(max_usable_range.unwrap_or(f64::INFINITY)) as f32;

        let lpose = self.parameter.offset * scan.odom_pose * scan.laser_params.laser_pose;
        let lcoords = self.world2map([lpose.translation.x, lpose.translation.y]);
        let path = {
            let mut pb = tiny_skia::PathBuilder::new();
            for (i, r) in scan
                .ranges
                .iter()
                .enumerate()
                .filter(|&x| *x.1 < usable_range)
            {
                pb.move_to(lcoords[0], lcoords[1]);
                let beam =
                    lpose * scan.laser_params.beam_isometry(i) * na::Point2::new(*r as f64, 0.);
                let coords = self.world2map([beam.x, beam.y]);
                pb.line_to(coords[0], coords[1]);
            }
            pb.finish().unwrap()
        };

        let mut paint = tiny_skia::Paint {
            anti_alias: true,
            ..Default::default()
        };
        paint.set_color_rgba8(242, 213, 207, 100);

        let stroke = tiny_skia::Stroke::default();

        self.img.stroke_path(
            &path,
            &paint,
            &stroke,
            tiny_skia::Transform::identity(),
            None,
        );
    }
}

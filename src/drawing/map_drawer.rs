use crate::datastream::robot_data::RobotLaser;
use image::RgbaImage;

pub struct MapDrawer {
    pub resolution: f64,
    pub offset: [f64; 2],
    pub img: tiny_skia::Pixmap,
}

impl MapDrawer {
    pub fn new(resolution: f64, offset: [f64; 2], img: tiny_skia::Pixmap) -> Self {
        Self {
            resolution,
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
            (wp[0] - self.offset[0]) / self.resolution,
            self.img.height() as f64 - (wp[1] - self.offset[1]) / self.resolution,
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
            let coords = self.world2map([
                scans[0].odom_pose.translation.x,
                scans[0].odom_pose.translation.y,
            ]);
            pb.move_to(coords[0], coords[1]);
            for s in scans.iter().skip(1) {
                let coords = self.world2map([s.odom_pose.translation.x, s.odom_pose.translation.y]);
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
}

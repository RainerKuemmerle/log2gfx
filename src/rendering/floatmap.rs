use super::gridmap;

use std::io::Write;
use std::path::PathBuf;
use std::{fs::File, io::BufWriter};

use image::RgbImage;

pub struct FloatMap {
    pub map: gridmap::GridMap<f32>,
}

fn color_for_occ(occ: f32) -> [u8; 3] {
    let c = 255. - 255. * occ;
    match occ {
        -1. => [140, 170, 238],
        -2. => [231, 130, 132],
        _ => [c as u8; 3],
    }
}

impl FloatMap {
    pub fn save_as_ppm(&self, filename: &PathBuf) -> std::io::Result<()> {
        let file = File::create(filename)?;
        let mut writer = BufWriter::new(file);

        writeln!(writer, "P6")?;
        writeln!(writer, "# log2gfx")?;
        writeln!(writer, "# resolution {}", self.map.resolution)?;
        writeln!(
            writer,
            "# offset {} {}",
            self.map.offset.x, self.map.offset.y
        )?;
        writeln!(writer, "{} {} 255", self.map.size[0], self.map.size[1])?;

        // Write the image content
        let default_cell = -1.0f32;
        for i in 1..=self.map.size[1] {
            let y = self.map.size[1] - i;
            for x in 0..self.map.size[0] {
                let occ = self.map.cell(x as i32, y as i32).unwrap_or(&default_cell);
                let rgb = color_for_occ(*occ);
                writer.write_all(&rgb)?;
            }
        }

        Ok(())
    }

    pub fn to_image(&self) -> RgbImage {
        let mut img = RgbImage::new(self.map.size[0] as u32, self.map.size[1] as u32);

        let default_cell = -1.0f32;
        for y in 0..self.map.size[1] {
            let ty = self.map.size[1] - y - 1;
            for x in 0..self.map.size[0] {
                let occ = self.map.cell(x as i32, y as i32).unwrap_or(&default_cell);
                let pixel = img.get_pixel_mut(x as u32, ty as u32);
                *pixel = image::Rgb(color_for_occ(*occ));
            }
        }

        img
    }
}

use super::gridmap;

use std::io::Write;
use std::path::PathBuf;
use std::{fs::File, io::BufWriter};

pub struct FloatMap {
    pub map: gridmap::GridMap<f32>,
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
                let c = 255. - 255. * occ;
                let rgb: [u8; 3] = match occ {
                    -1. => [140, 170, 238],
                    -2. => [231, 130, 132],
                    _ => [c as u8; 3],
                };
                writer.write_all(&rgb)?;
            }
        }

        Ok(())
    }
}

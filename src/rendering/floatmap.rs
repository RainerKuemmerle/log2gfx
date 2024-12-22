use super::gridmap;

use std::io::Write;
use std::{fs::File, io::BufWriter};

pub struct FloatMap {
    pub map: gridmap::GridMap<f32>,
}

impl FloatMap {
    pub fn save_as_ppm(&self, filename: &str) -> std::io::Result<()> {
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
        writeln!(writer, "{} {} 255", self.map.size.x, self.map.size.y)?;

        // Write the image content
        for occ in self.map.cells() {
            let c = 255. - 255. * occ;
            let rgb: [u8; 3];
            match occ {
                -1. => rgb = [210, 190, 190],
                -2. => rgb = [64, 64, 255],
                -3. => rgb = [255, 64, 64],
                &_ => rgb = [c as u8; 3],
            }
            writer.write(&rgb)?;
        }

        Ok(())
    }
}

use super::gridmap;

pub struct FloatMap {
    pub map: gridmap::GridMap<f32>,
}

fn color_for_occ(occ: f32) -> [u8; 4] {
    let c = (255. - 255. * occ) as u8;
    match occ {
        -1. => [140, 170, 238, 255],
        _ => [c, c, c, 255],
    }
}

impl FloatMap {
    pub fn to_pixels(&self) -> Vec<u8> {
        let capacity = self.map.size[0] * self.map.size[1] * 4;
        let mut img = Vec::with_capacity(capacity);

        let default_cell = -1.0f32;
        for i in 1..=self.map.size[1] {
            let y = self.map.size[1] - i;
            for x in 0..self.map.size[0] {
                let occ = self.map.cell(x as i32, y as i32).unwrap_or(&default_cell);
                img.extend(color_for_occ(*occ));
            }
        }

        img
    }
}

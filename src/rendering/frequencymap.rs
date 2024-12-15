use super::gridmap;

#[derive(Debug, Copy, Clone)]
struct FrequencyMapCell {
    hits: i32,
    misses: i32,
}

pub struct FrequencyMap {
    map: gridmap::GridMap<FrequencyMapCell>,
}

impl FrequencyMap {
    pub fn new(size: na::Vector2<usize>, resolution: f64, offset: na::Vector2<f64>) -> Self {
        let default_cell = FrequencyMapCell { hits: 0, misses: 0 };
        let map = gridmap::GridMap::new(size, resolution, offset, default_cell);
        Self { map }
    }
}

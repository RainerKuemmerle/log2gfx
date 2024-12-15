extern crate nalgebra as na;

pub struct GridMap<T> {
    pub resolution: f64,
    pub offset: na::Vector2<f64>,
    pub size: na::Vector2<usize>,
    grid: Vec<T>,
}

impl<T: Copy> GridMap<T> {
    pub fn new(
        size: na::Vector2<usize>,
        resolution: f64,
        offset: na::Vector2<f64>,
        unknown_cell: T,
    ) -> Self {
        let grid = vec![unknown_cell; size.x * size.y];
        Self {
            resolution,
            offset,
            size,
            grid,
        }
    }

    pub fn cell_scalar(&mut self, x: usize, y: usize) -> Option<&mut T> {
        self.grid.get_mut(y * self.size.x + x)
    }

    pub fn cell(&mut self, p: &na::Vector2<i32>) -> Option<&mut T> {
        if p.x < 0 || p.y < 0 {
            None
        } else {
            self.cell_scalar(p.x as usize, p.y as usize)
        }
    }

    pub fn world2map(&self, wp: &na::Vector2<f64>) -> na::Vector2<i32> {
        let map_point = (wp - self.offset) / self.resolution;
        na::Vector2::new(map_point.x as i32, map_point.y as i32)
    }

    pub fn map2world(&self, mp: &na::Vector2<i32>) -> na::Vector2<f64> {
        self.offset + ((mp.cast::<f64>() + na::Vector2::new(0.5, 0.5)) * self.resolution)
    }

    pub fn is_inside(&self, mp: &na::Vector2<i32>) -> bool {
        mp.x > 0 && mp.y > 0 && (mp.x as usize) < self.size.x && (mp.y as usize) < self.size.y
    }
}

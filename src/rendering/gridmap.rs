extern crate nalgebra as na;

pub struct GridMap<T> {
    pub resolution: f64,
    pub offset: na::Vector2<f64>,
    pub size: [usize; 2],
    grid: Vec<T>,
}

impl<T: Copy> GridMap<T> {
    pub fn new(
        size: [usize; 2],
        resolution: f64,
        offset: na::Vector2<f64>,
        unknown_cell: T,
    ) -> Self {
        let grid = vec![unknown_cell; size[0] * size[1]];
        Self {
            resolution,
            offset,
            size,
            grid,
        }
    }

    pub fn cells(&self) -> impl Iterator<Item = &T> {
        self.grid.iter()
    }

    pub fn cells_mut(&mut self) -> impl Iterator<Item = &mut T> {
        self.grid.iter_mut()
    }

    pub fn cell_mut(&mut self, x: i32, y: i32) -> Option<&mut T> {
        if !self.is_inside(x, y) {
            None
        } else {
            self.grid.get_mut(y as usize * self.size[0] + x as usize)
        }
    }

    pub fn cell(&self, x: i32, y: i32) -> Option<&T> {
        if !self.is_inside(x, y) {
            None
        } else {
            self.grid.get(y as usize * self.size[0] + x as usize)
        }
    }

    pub fn world2map(&self, wp: &na::Vector2<f64>) -> na::Vector2<i32> {
        let map_point = (wp - self.offset) / self.resolution;
        na::Vector2::new(map_point.x as i32, map_point.y as i32)
    }

    pub fn is_inside(&self, x: i32, y: i32) -> bool {
        x > 0 && y > 0 && (x as usize) < self.size[0] && (y as usize) < self.size[1]
    }
}

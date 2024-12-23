pub struct Bresenham {
    end_x: i32,
    end_y: i32,
    dx: i32,
    dy: i32,
    sx: i32,
    sy: i32,
    error: i32,
    current_x: i32,
    current_y: i32,
    consumed: bool,
}

impl Iterator for Bresenham {
    type Item = [i32; 2];
    fn next(&mut self) -> Option<Self::Item> {
        if self.consumed {
            return None;
        }
        let result = [self.current_x, self.current_y];
        if self.current_x == self.end_x && self.current_y == self.end_y {
            self.consumed = true;
        } else {
            let error2 = self.error;
            if error2 > -self.dx {
                self.error -= self.dy;
                self.current_x += self.sx;
            }
            if error2 < self.dy {
                self.error += self.dx;
                self.current_y += self.sy;
            }
        }
        Some(result)
    }
}

pub fn bresenham(x1: i32, y1: i32, x2: i32, y2: i32) -> Bresenham {
    let dx = (x2 - x1).abs();
    let dy = (y2 - y1).abs();
    Bresenham {
        end_x: x2,
        end_y: y2,
        dx,
        dy,
        sx: if x1 < x2 { 1 } else { -1 },
        sy: if y1 < y2 { 1 } else { -1 },
        error: (if dx > dy { dx } else { -dy }) / 2,
        current_x: x1,
        current_y: y1,
        consumed: false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bresenham_simple() {
        let mut line = bresenham(0, 0, 3, 0);
        assert_eq!(line.next().unwrap(), [0, 0]);
        assert_eq!(line.next().unwrap(), [1, 0]);
        assert_eq!(line.next().unwrap(), [2, 0]);
        assert_eq!(line.next().unwrap(), [3, 0]);
        assert_eq!(line.next(), None);
    }
}

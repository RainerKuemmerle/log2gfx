pub struct Bresenham {
    end: [i32; 2],
    delta: [i32; 2],
    step: [i32; 2],
    error: i32,
    current: [i32; 2],
    consumed: bool,
}

impl Iterator for Bresenham {
    type Item = [i32; 2];
    fn next(&mut self) -> Option<Self::Item> {
        if self.consumed {
            return None;
        }
        let result = self.current;
        if self.current == self.end {
            self.consumed = true;
        } else {
            let error2 = self.error;
            if error2 > -self.delta[0] {
                self.error -= self.delta[1];
                self.current[0] += self.step[0];
            }
            if error2 < self.delta[1] {
                self.error += self.delta[0];
                self.current[1] += self.step[1];
            }
        }
        Some(result)
    }
}

pub fn bresenham(x1: i32, y1: i32, x2: i32, y2: i32) -> Bresenham {
    let dx = (x2 - x1).abs();
    let dy = (y2 - y1).abs();
    Bresenham {
        end: [x2, y2],
        delta: [dx, dy],
        step: [if x1 < x2 { 1 } else { -1 }, if y1 < y2 { 1 } else { -1 }],
        error: (if dx > dy { dx } else { -dy }) / 2,
        current: [x1, y1],
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

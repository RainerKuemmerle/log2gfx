pub fn bresenham(x1: i32, y1: i32, x2: i32, y2: i32) -> Vec<[i32; 2]> {
    let mut coordinates = vec![];
    let dx: i32 = (x2 - x1).abs();
    let dy: i32 = (y2 - y1).abs();
    let sx: i32 = if x1 < x2 { 1 } else { -1 };
    let sy: i32 = if y1 < y2 { 1 } else { -1 };

    let mut error: i32 = (if dx > dy { dx } else { -dy }) / 2;
    let mut current_x: i32 = x1;
    let mut current_y: i32 = y1;
    loop {
        coordinates.push([current_x, current_y]);

        if current_x == x2 && current_y == y2 {
            break;
        }

        let error2: i32 = error;

        if error2 > -dx {
            error -= dy;
            current_x += sx;
        }
        if error2 < dy {
            error += dx;
            current_y += sy;
        }
    }
    coordinates
}

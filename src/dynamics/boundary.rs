use geo::MapCoordsInPlace;

fn wrap1(x: f64, l: f64) -> f64 {
    if x < -l * 0.5 {
        x + l
    } else if x > l * 0.5 {
        x - l
    } else {
        x
    }
}

pub fn wrap(r: &mut geo::Point, l: f64) {
    r.map_coords_in_place(|geo::Coord { x, y }| geo::Coord {
        x: wrap1(x, l),
        y: wrap1(y, l),
    });
}

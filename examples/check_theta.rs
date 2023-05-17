use std::{assert_eq, collections::HashMap};

#[derive(Debug)]
struct Item {
    i: usize,
    index: usize,
    r: f64,
    x: f64,
    y: f64,
    z: f64,
    theta: f64,
    ring_idx: i64,
    sector_idx: i64,
}

impl PartialEq for Item {
    fn eq(&self, other: &Self) -> bool {
        fn ef(a: f64, b: f64) -> bool {
            (a - b).abs() < 0.0001
        }
        self.i == other.i
            && self.index == other.index
            && self.ring_idx == other.ring_idx
            && self.sector_idx == other.sector_idx
            && ef(self.x, other.x)
            && ef(self.y, other.y)
            && ef(self.z, other.z)
            && ef(self.r, other.r)
            && ef(self.theta, other.theta)
    }
}

impl Eq for Item {}

fn main() {
    let rust = parse_file("rust_theta.log");
    let cpp = parse_file("cpp_theta.log");

    for (index, item) in &rust {
        let other = cpp.get(&index).expect(&format!("id = {}", index));
        assert_eq!(item, other);
    }

    // for (index, item) in &cpp {
    //     let other = rust.get(&index).expect(&format!("id = {}", index));
    //     assert_eq!(item, other);
    // }
}

fn parse_file(file: &str) -> HashMap<usize, Item> {
    std::fs::read_to_string(file)
        .unwrap()
        .lines()
        .map(|line| {
            let x = parse_line(line);
            (x.i, x)
        })
        .collect()
}

fn parse_line(line: &str) -> Item {
    let value: serde_json::Value = serde_json::from_str(line).unwrap();
    let i = value.get("i").unwrap().as_i64().unwrap() as usize;
    Item {
        i,
        index: value.get("index").unwrap().as_i64().unwrap() as usize,
        r: value.get("r").unwrap().as_f64().unwrap(),
        x: value.get("x").unwrap().as_f64().unwrap(),
        y: value.get("y").unwrap().as_f64().unwrap(),
        z: value
            .get("z")
            .expect(&format! {"i = {i}"})
            .as_f64()
            .expect(&format! {"i = {i}"}),
        theta: value.get("theta").unwrap().as_f64().unwrap(),
        ring_idx: value.get("ring_idx").unwrap().as_i64().unwrap(),
        sector_idx: value.get("sector_idx").unwrap().as_i64().unwrap(),
    }
}

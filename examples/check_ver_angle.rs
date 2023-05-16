use std::println;

#[derive(Debug)]
struct Item {
    deg: f64,
}

impl PartialEq for Item {
    fn eq(&self, other: &Self) -> bool {
        fn ef(a: f64, b: f64) -> bool {
            a - b < 0.0001
        }
        ef(self.deg, other.deg)
    }
}

impl Eq for Item {}

fn main() {
    let rust = parse_file("rust_ver_angle_in_deg.log");
    let cpp = parse_file("cpp_ver_angle_in_deg.log");

    assert_eq!(rust.len(), cpp.len());

    for (r, c) in rust.into_iter().zip(&cpp) {
        assert_eq!(r, *c);
    }
}

fn parse_file(file: &str) -> Vec<Item> {
    std::fs::read_to_string(file)
        .unwrap()
        .lines()
        .map(|line| parse_line(line))
        .collect()
}

fn parse_line(line: &str) -> Item {
    let mut iter = line.split_whitespace().filter(|x| !x.trim().is_empty());
    assert_eq!(iter.next().unwrap(), "ver_angle_in_deg:");
    let deg = iter.next().unwrap().parse::<f64>().unwrap();

    Item { deg }
}

use std::println;

#[derive(Debug)]
struct Item {
    i: usize,
    x: f64,
    y: f64,
    z: f64,
}

impl PartialEq for Item {
    fn eq(&self, other: &Self) -> bool {
        fn ef(a: f64, b: f64) -> bool {
            a - b < 0.0001
        }
        self.i == other.i && ef(self.x, other.x) && ef(self.y, other.y) && ef(self.z, other.z)
    }
}

impl Eq for Item {}

fn main() {
    // let v: f64 = (2.0f64 / 3.0).atan();
    // println!("(2.0 / 3.0).atan() = {}", v);
    // let v: f64 = (2.0f64).atan2(3.0);
    // println!("(2.0).atan2(3.0) = {}", v);

    return;
    let rust = parse_file("rust.log");
    let cpp = parse_file("cpp.log");

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
    assert_eq!(iter.next().unwrap(), "i");
    assert_eq!(iter.next().unwrap(), "=");
    let i = iter.next().unwrap().parse::<usize>().unwrap();

    assert_eq!(iter.next().unwrap(), "x");
    assert_eq!(iter.next().unwrap(), "=");
    let x = iter.next().unwrap().parse::<f64>().unwrap();

    assert_eq!(iter.next().unwrap(), "y");
    assert_eq!(iter.next().unwrap(), "=");
    let y = iter.next().unwrap().parse::<f64>().unwrap();

    assert_eq!(iter.next().unwrap(), "z");
    assert_eq!(iter.next().unwrap(), "=");
    let z = iter.next().unwrap().parse::<f64>().unwrap();

    Item { i, x, y, z }
}

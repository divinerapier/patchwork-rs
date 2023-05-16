use std::{io::Read, os::unix::prelude::MetadataExt, println};

use patchwork::{Params, PatchWorkpp};

fn main() {
    sequential();
    println!("hello world")
}

fn sequential() {
    dotenv::dotenv().unwrap();
    // println!("{:?}", std::env::var("RUST_MIN_STACK"));
    fn read_bin(path: &str) {
        let mut file = std::fs::File::open(path).unwrap();
        let size = file.metadata().unwrap().size();
        assert!(size % 4 == 0);
        let mut buffer = vec![];
        let result = file.read_to_end(&mut buffer).unwrap();

        // file.read_exact(&mut buffer).unwrap();
        // println!("buffer = {:?}", &buffer[..100]);
        let points = {
            let len = buffer.len() / 4;
            unsafe { Vec::from_raw_parts(buffer.as_mut_ptr() as *mut f32, len, len) }
        };
        // let points = unsafe { std::mem::transmute::<Vec<u8>, Vec<f32>>(buffer) };
        // println!("points = {:?}", &points[..25]);

        let num_points = points.len() / 4;
        // println!("num_points = {:?}", num_points);

        let mut mat = nalgebra::MatrixXx4::<f64>::zeros(num_points);
        // super::debug_matrix("mat", &mat);
        for i in 0..num_points {
            mat[(i, 0)] = points[i * 4] as f64;
            mat[(i, 1)] = points[i * 4 + 1] as f64;
            mat[(i, 2)] = points[i * 4 + 2] as f64;
            mat[(i, 3)] = points[i * 4 + 3] as f64;
        }
        // patchwork::debug_matrix("mat", &mat);
        println!("buffer.len() = {:?}, result = {}", buffer.len(), result);
        println!("size = {:?}", size);
        println!("points.len() = {}", points.len());
        println!("num points: {}", num_points);
        {
            let params = Params::default();
            let mut patchworkpp = PatchWorkpp::new(params);
            patchworkpp.estimate_ground(&mut mat);
        }
        assert!(false)
    }

    read_bin("data/000002.bin");
}
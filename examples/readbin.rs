use std::{io::Read, os::unix::prelude::MetadataExt, println};

use nalgebra::{Const, Dyn, Dynamic, VecStorage};
use patchwork::{Params, PatchWorkpp};

fn main() {
    sequential();
    println!("hello world");
    assert!(true);
}

// i = 115929 r = 13.750170634906302 z = -5.282143592834473 ver_angle_in_deg = -21.01433542566051
// i = 115930 r = 13.783687151198826 z = -5.295128345489502 ver_angle_in_deg = -21.014731498586254
// i = 118072 r = 26.416925607528498 z = -11.03048324584961 ver_angle_in_deg = -22.663153042742152

fn sequential() {
    dotenv::dotenv().unwrap();
    // println!("{:?}", std::env::var("RUST_MIN_STACK"));

    read_bin("data/000002.bin");
    println!("xxx");
}

fn read_bin(path: &str) {
    let mut file = std::fs::File::open(path).unwrap();
    let size = file.metadata().unwrap().size();
    assert!(size % 4 == 0);
    let mut buffer = vec![];
    let result = file.read_to_end(&mut buffer).unwrap();
    let buffer_length = buffer.len();
    // file.read_exact(&mut buffer).unwrap();
    // println!("buffer = {:?}", &buffer[..100]);
    let points = {
        let len = buffer_length / 4;
        unsafe {
            let v = Vec::from_raw_parts(buffer.as_mut_ptr() as *mut f32, len, len);
            std::mem::forget(buffer);
            v
        }
    };

    // let points = unsafe { std::mem::transmute::<Vec<u8>, Vec<f32>>(buffer) };
    // println!("points = {:?}", &points[..25]);

    let num_points = points.len() / 4;
    // println!("num_points = {:?}", num_points);

    let mut mat: nalgebra::Matrix<f32, Dyn, Const<4>, nalgebra::VecStorage<f32, Dyn, Const<4>>> =
        nalgebra::MatrixXx4::<f32>::zeros(num_points);
    // // super::debug_matrix("mat", &mat);
    for i in 0..num_points {
        mat[(i, 0)] = points[i * 4] as f32;
        mat[(i, 1)] = points[i * 4 + 1] as f32;
        mat[(i, 2)] = points[i * 4 + 2] as f32;
        mat[(i, 3)] = points[i * 4 + 3] as f32;
        // println!(
        //     "i = {} x = {} y = {} z = {}",
        //     i,
        //     points[i * 4] as f64,
        //     points[i * 4 + 1] as f64,
        //     points[i * 4 + 2] as f64
        // );
    }

    // 这里的想法是直接使用转换后的 vector 作为 matrix 的底层存储。但是很遗憾的是，
    // nalgebra 使用列优先数据结构，即列数据是相邻的。而我们的 vector 是行优先的，即行数据是相邻的。
    // 一个可行的解决方案，上游数据来源是列村结构。所以，所以无法避免需要执行一次 memory copy。
    //
    // 以下内容来自 https://docs.rs/nalgebra/latest/nalgebra/base/struct.Matrix.html#method.index
    //
    // # Views based on ranges
    // ## Indices to Individual Elements
    // ### Two-Dimensional Indices
    // ```
    // # use nalgebra::*;
    // let matrix = Matrix2::new(0, 2,
    //                           1, 3);
    //
    // assert_eq!(matrix.index((0, 0)), &0);
    // assert_eq!(matrix.index((1, 0)), &1);
    // assert_eq!(matrix.index((0, 1)), &2);
    // assert_eq!(matrix.index((1, 1)), &3);
    // ```
    //
    // ### Linear Address Indexing
    // ```
    // # use nalgebra::*;
    // let matrix = Matrix2::new(0, 2,
    //                           1, 3);
    //
    // assert_eq!(matrix.get(0), Some(&0));
    // assert_eq!(matrix.get(1), Some(&1));
    // assert_eq!(matrix.get(2), Some(&2));
    // assert_eq!(matrix.get(3), Some(&3));
    // ```
    //
    // let mut mak: nalgebra::Matrix<f32, Dyn, Const<4>, nalgebra::VecStorage<f32, Dyn, Const<4>>> =
    //     nalgebra::MatrixXx4::<f32>::zeros(0);

    // let row = Dyn(num_points);
    // let col = Const::<4>;

    // mak.data = VecStorage::<f32, Dyn, Const<4>>::new(row, col, points);

    // assert_eq!(mat.len(), mak.len());

    // for i in 0..mat.len() {
    //     let left = (
    //         mat[i],
    //         mat[i + 1],
    //         mat[i + 2],
    //         mat[i + 3],
    //         mat[i + 4],
    //         mat[i + 5],
    //         mat[i + 6],
    //         mat[i + 7],
    //         mat[i + 8],
    //     );
    //     let right = (
    //         mak[i],
    //         mak[i + 1],
    //         mak[i + 2],
    //         mak[i + 3],
    //         mak[i + 4],
    //         mak[i + 5],
    //         mak[i + 6],
    //         mak[i + 7],
    //         mak[i + 8],
    //     );

    //     assert_eq!(left, right);
    // }

    // for i in 0..mat.len() {
    //     let left = mat.row(i);
    //     let left = (left[0], left[1], left[2], left[3]);
    //     let right = mak.row(i);
    //     let right = (right[0], right[1], right[2], right[3]);

    //     assert_eq!(left, right);
    // }

    // patchwork::debug_matrix("mat", &mat);
    println!("buffer.len() = {:?}, result = {}", buffer_length, result);
    println!("size = {:?}", size);
    // println!("points.len() = {}", points.len());
    println!("num points: {}", num_points);
    let params = Params::default();
    let mut patchworkpp = PatchWorkpp::new(params);
    patchworkpp.estimate_ground(&mut mat);
    let ground = patchworkpp.get_ground();
    println!("ground = ",);
    for row in ground.row_iter() {
        println!("{} {} {}", row[0], row[1], row[2]);
        return;
    }
}

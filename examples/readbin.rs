use std::{io::Read, os::unix::prelude::MetadataExt, println};

use patchwork::{Params, PatchWork};

fn main() {
    sequential();
    assert!(true);
}

fn sequential() {
    dotenv::dotenv().unwrap();

    read_bin("data/000002.bin");
}

fn read_bin(path: &str) {
    let mut file = std::fs::File::open(path).unwrap();
    let size = file.metadata().unwrap().size();
    assert!(size % 4 == 0);
    let mut buffer = vec![];
    let result = file.read_to_end(&mut buffer).unwrap();
    let buffer_length = buffer.len();

    let mut mat = patchwork::matrix_from_raw_buffer_drop_last::<3>(buffer);

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

    println!("buffer.len() = {:?}, result = {}", buffer_length, result);
    println!("size = {:?}", size);
    println!("num points: {}", mat.len());
    let params = Params::default();
    let patchworkpp = PatchWork::new(params);
    let ground = patchworkpp.estimate_ground(&mut mat);

    {
        let g = ground.get_ground();
        println!("ground = ",);
        println!("rows = {}", g.len() / 3);

        let first = g.row(0);
        println!("{} {} {}", first[0], first[1], first[2]);

        let row = g.row(30000);
        println!("{} {} {}", row[0], row[1], row[2]);

        let last = g.row_iter().last().unwrap();
        println!("{} {} {}", last[0], last[1], last[2]);

        let row_sum = g.row_sum();
        println!("{} {} {}", row_sum[0], row_sum[1], row_sum[2]);
    }
    {
        let g = ground.get_nonground();
        println!("non ground = ",);
        println!("rows = {}", g.len() / 3);

        let first = g.row(0);
        println!("{} {} {}", first[0], first[1], first[2]);

        let row = g.row(30000);
        println!("{} {} {}", row[0], row[1], row[2]);

        let last = g.row_iter().last().unwrap();
        println!("{} {} {}", last[0], last[1], last[2]);

        let row_sum = g.row_sum();
        println!("{} {} {}", row_sum[0], row_sum[1], row_sum[2]);
    }
}

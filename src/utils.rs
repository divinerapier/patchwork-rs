use std::f64::consts::PI;

use nalgebra::{Dim, OVector, RawStorage};

use crate::Point3D;

pub(crate) fn calc_point_to_plane_d(
    point: &Point3D,
    normal: &OVector<f32, nalgebra::Const<3>>,
    d: f64,
) -> f64 {
    normal[0] as f64 * point.x as f64
        + normal[1] as f64 * point.y as f64
        + normal[2] as f64 * point.z as f64
        + d
}

pub(crate) fn xy2radius(x: f64, y: f64) -> f64 {
    (x * x + y * y).sqrt()
}

pub(crate) fn xy2theta(x: f64, y: f64) -> f64 {
    let angle = y.atan2(x);
    if angle > 0. {
        angle
    } else {
        angle + 2.0 * PI
    }
}
pub fn debug_matrix<
    T: std::fmt::Debug + std::fmt::Display,
    R: Dim,
    C: Dim,
    S: std::fmt::Debug + RawStorage<T, R, C>,
>(
    name: &str,
    matrix: &nalgebra::Matrix<T, R, C, S>,
) {
    return;
    println!("debug matrix [{name}]:",);
    for (index, row) in matrix.row_iter().enumerate() {
        print!("[");
        for value in row.iter() {
            print!("{} ", value);
        }
        println!("]");
    }
    println!("");
}

pub trait IntoSplitFilter {
    type Item;

    fn into_split_filter<P>(self, predict: P) -> SplitFilter<Self, P>
    where
        Self: Sized,
        P: FnMut(&Self::Item) -> bool,
    {
        SplitFilter::new(self, predict)
    }
}

pub struct SplitFilter<I, P> {
    iter: I,
    predict: P,
}

impl<I, P> SplitFilter<I, P> {
    pub fn new(iter: I, predict: P) -> Self {
        Self { iter, predict }
    }
}

impl<T> IntoSplitFilter for T
where
    T: Iterator,
{
    type Item = T::Item;
}

impl<I, P> Iterator for SplitFilter<I, P>
where
    I: Iterator,
    P: FnMut(&I::Item) -> bool,
{
    type Item = (Option<I::Item>, Option<I::Item>);

    fn next(&mut self) -> Option<Self::Item> {
        let value = self.iter.next()?;
        let predict = &mut self.predict;
        if predict(&value) {
            Some((Some(value), None))
        } else {
            Some((None, Some(value)))
        }
    }
}

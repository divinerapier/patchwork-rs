use std::ops::{Deref, DerefMut, Index};

use crate::IntoCloud;

#[derive(Debug, Clone, Copy)]
pub struct Point3D {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Point3D {
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    pub fn compare_z(&self, rhs: &Point3D) -> std::cmp::Ordering {
        self.z
            .partial_cmp(&rhs.z)
            .unwrap_or(std::cmp::Ordering::Equal)
    }
}

#[derive(Debug, Clone)]
pub struct Points(Vec<Point3D>);

impl Deref for Points {
    type Target = Vec<Point3D>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for Points {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

#[derive(Debug, Clone)]
pub struct Ring(Vec<Points>);

impl Deref for Ring {
    type Target = Vec<Points>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for Ring {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

#[derive(Debug, Clone)]
pub struct Zone(Vec<Ring>);

impl Deref for Zone {
    type Target = Vec<Ring>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for Zone {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

// impl Index<usize> for Points {
//     type Output = Point3D;

//     fn index(&self, index: usize) -> &Self::Output {
//         &self.0[index]
//     }
// }

// impl Index<usize> for Ring {
//     type Output = Points;

//     fn index(&self, index: usize) -> &Self::Output {
//         &self.0[index]
//     }
// }

// impl Index<usize> for Zone {
//     type Output = Ring;

//     fn index(&self, index: usize) -> &Self::Output {
//         &self.0[index]
//     }
// }

impl Default for Zone {
    fn default() -> Self {
        Self(Vec::new())
    }
}

impl AsRef<[Point3D]> for Points {
    fn as_ref(&self) -> &[Point3D] {
        self.0.as_ref()
    }
}

impl Points {
    pub fn add_cloud<P: AsRef<[Point3D]>>(&mut self, points: P) {
        self.0.extend(points.as_ref())
    }
}

impl From<Vec<Point3D>> for Points {
    fn from(points: Vec<Point3D>) -> Self {
        Self(points)
    }
}

impl From<Vec<Points>> for Ring {
    fn from(points: Vec<Points>) -> Self {
        Self(points)
    }
}

impl From<Vec<Ring>> for Zone {
    fn from(points: Vec<Ring>) -> Self {
        Self(points)
    }
}

// pub fn add_cloud(cloud: &mut Vec<PointXYZ>, points: &[PointXYZ]) {
//     cloud.extend(points);
// }

// pub fn compare_point_z(a: &PointXYZ, b: &PointXYZ) -> std::cmp::Ordering {
//     a.z.partial_cmp(&b.z).unwrap_or(std::cmp::Ordering::Equal)
// }

impl IntoCloud for &[Point3D] {
    fn into_cloud(self) -> nalgebra::MatrixXx3<f32> {
        let mut cloud = nalgebra::MatrixXx3::zeros(self.len());

        for (i, p) in self.iter().enumerate() {
            cloud[(i, 0)] = p.x;
            cloud[(i, 1)] = p.y;
            cloud[(i, 2)] = p.z;
        }

        cloud
    }
}

impl IntoCloud for &Points {
    fn into_cloud(self) -> nalgebra::MatrixXx3<f32> {
        let mut cloud = nalgebra::MatrixXx3::zeros(self.as_ref().len());

        for (i, p) in self.as_ref().iter().enumerate() {
            cloud[(i, 0)] = p.x;
            cloud[(i, 1)] = p.y;
            cloud[(i, 2)] = p.z;
        }

        cloud
    }
}

#[test]
fn test_points_to_cloud() {
    use super::*;

    let points = vec![
        Point3D::new(1.0, 2.0, 3.0),
        Point3D::new(4.0, 5.0, 6.0),
        Point3D::new(7.0, 8.0, 9.0),
        Point3D::new(11.0, 12.0, 13.0),
        Point3D::new(14.0, 15.0, 16.0),
        Point3D::new(17.0, 18.0, 19.0),
    ];

    let expected = vec![
        vec![1. as f32, 2., 3.],
        vec![4., 5., 6.],
        vec![7., 8., 9.],
        vec![11., 12., 13.],
        vec![14., 15., 16.],
        vec![17., 18., 19.],
    ];

    let cloud = points.into_cloud();
    for (id, row) in cloud.row_iter().enumerate() {
        assert_eq!(3, expected[id].len());
        assert_eq!(expected[id].len(), row.len());
        for (i, v) in row.iter().enumerate() {
            assert_eq!(expected[id][i], *v);
        }
    }
}

#[test]
fn test_add_cloud() {
    use super::*;

    let mut u = Points::from(vec![
        Point3D::new(1.0, 2.0, 3.0),
        Point3D::new(4.0, 5.0, 6.0),
        Point3D::new(7.0, 8.0, 9.0),
    ]);

    let v = Points::from(vec![
        Point3D::new(11.0, 12.0, 13.0),
        Point3D::new(14.0, 15.0, 16.0),
        Point3D::new(17.0, 18.0, 19.0),
    ]);

    u.add_cloud(&v);

    let expected = vec![
        vec![1. as f32, 2., 3.],
        vec![4., 5., 6.],
        vec![7., 8., 9.],
        vec![11., 12., 13.],
        vec![14., 15., 16.],
        vec![17., 18., 19.],
    ];

    let cloud = u.into_cloud();
    for (id, row) in cloud.row_iter().enumerate() {
        assert_eq!(3, expected[id].len());
        assert_eq!(expected[id].len(), row.len());
        for (i, v) in row.iter().enumerate() {
            assert_eq!(expected[id][i], *v);
        }
    }
}

#[test]
fn test_estimate_plane() {
    use super::*;

    let params = Params::default();

    let mut workpp = PatchWorkpp::new(params);

    // let dst = vec![
    //     Point3D::new(1.0, 2.0, 3.0),
    //     Point3D::new(4.0, 5.0, 6.0),
    //     Point3D::new(7.0, 8.0, 9.0),
    //     Point3D::new(11.0, 12.0, 13.0),
    //     Point3D::new(14.0, 15.0, 16.0),
    //     Point3D::new(17.0, 18.0, 19.0),
    // ];

    // workpp.estimate_plane(patchworkpp::EstimatePlaneTarget::Other(dst.into()));
    {
        // cov 矩阵
        // [0.07714679 -0.015178974 0.007949526 ]
        // [-0.015178974 0.096720874 -0.01979263 ]
        // [0.007949526 -0.01979263 0.08639938 ]

        // 得到 singular_values
        // [0.11886798 ]
        // [0.07349675 ]
        // [0.06790227 ]

        // U
        // [0.37624383 -0.60235226 0.70399725 ]
        // [-0.74732035 0.25185028 0.6148851 ]
        // [0.5476793 0.7574584 0.35539293 ]

        // Vt
        // [0.37624404 -0.7473201 0.5476793 ]
        // [-0.6023522 0.25185034 0.7574583 ]
        // [0.7039972 0.6148851 0.35539293 ]
        let params = Params::default();

        let mut workpp = PatchWorkpp::new(params);

        let dst = vec![
            Point3D::new(0.186, 0.053, 0.468),
            Point3D::new(0.170, 0.768, 0.657),
            Point3D::new(0.501, 0.319, 0.051),
            Point3D::new(0.635, 0.549, 0.628),
            Point3D::new(0.240, 0.879, 0.598),
            Point3D::new(0.988, 0.364, 0.750),
            Point3D::new(0.482, 0.091, 0.423),
            Point3D::new(0.372, 0.842, 0.803),
            Point3D::new(0.729, 0.277, 0.547),
            Point3D::new(0.473, 0.480, 0.190),
            Point3D::new(0.106, 0.450, 0.266),
            Point3D::new(0.823, 0.991, 0.396),
            Point3D::new(0.583, 0.042, 0.522),
            Point3D::new(0.913, 0.786, 0.046),
            Point3D::new(0.660, 0.233, 0.889),
            Point3D::new(0.526, 0.620, 0.108),
            Point3D::new(0.791, 0.139, 0.312),
            Point3D::new(0.253, 0.921, 0.060),
            Point3D::new(0.010, 0.763, 0.104),
            Point3D::new(0.369, 0.306, 0.988),
        ];

        workpp.estimate_plane(patchworkpp::EstimatePlaneTarget::Other(dst.into()));
    }
}

#[test]
fn erase_vector() {
    {
        let mut v = vec![0, 1, 2, 3, 4];
        let start = 0;
        let num = 3;
        v.drain(start..start + num);
        assert_eq!(vec![3, 4], v);
    }
    {
        let mut v = vec![0, 1, 2, 3, 4];
        let start = 0;
        let num = 3;
        v.drain(start + 1..start + num);
        assert_eq!(vec![0, 3, 4], v);
    }
}

#[test]
fn test_math() {
    use crate::utils::{xy2radius, xy2theta};

    let x = 5.0;
    let y = 6.0;

    let r = xy2radius(x, y);
    let theta = xy2theta(x, y);

    assert_eq!(r, 7.810249675906654);
    assert_eq!(theta, 0.8760580505981934);
}

#[test]
fn test_estimate_ground() {
    use super::*;

    const N: usize = 3;

    let params = Params::default();

    let points = vec![
        Point3D::new(186., 53., 468.),
        Point3D::new(170., 768., 657.),
        Point3D::new(501., 319., 51.),
        Point3D::new(635., 549., 628.),
        Point3D::new(240., 879., 598.),
        Point3D::new(988., 364., 750.),
        Point3D::new(482., 91., 423.),
        Point3D::new(372., 842., 803.),
        Point3D::new(729., 277., 547.),
        Point3D::new(473., 480., 190.),
        Point3D::new(106., 450., 266.),
        Point3D::new(823., 991., 396.),
        Point3D::new(583., 042., 522.),
        Point3D::new(913., 786., 46.),
        Point3D::new(660., 233., 889.),
        Point3D::new(526., 620., 108.),
        Point3D::new(791., 139., 312.),
        Point3D::new(253., 921., 60.),
        Point3D::new(10., 763., 104.),
        Point3D::new(369., 306., 988.),
    ];

    let mut workpp = PatchWorkpp::new(params);

    let mut mat: nalgebra::Matrix<
        f32,
        nalgebra::Dyn,
        nalgebra::Const<N>,
        nalgebra::VecStorage<f32, nalgebra::Dyn, nalgebra::Const<N>>,
    > = nalgebra::Matrix::<f32, nalgebra::Dyn, nalgebra::Const<N>, _>::zeros(points.len());
    for i in 0..points.len() {
        mat[(i, 0)] = points[i].x;
        mat[(i, 1)] = points[i].y;
        mat[(i, 2)] = points[i].z;
    }
    workpp.estimate_ground(&mut mat);

    assert_eq!(workpp.d, 6.94197e-310);
}

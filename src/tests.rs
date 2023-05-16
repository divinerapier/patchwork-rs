#[test]
fn test_points_to_cloud() {
    use super::*;

    let points = vec![
        PointXYZ::new(1.0, 2.0, 3.0),
        PointXYZ::new(4.0, 5.0, 6.0),
        PointXYZ::new(7.0, 8.0, 9.0),
        PointXYZ::new(11.0, 12.0, 13.0),
        PointXYZ::new(14.0, 15.0, 16.0),
        PointXYZ::new(17.0, 18.0, 19.0),
    ];

    let expected = vec![
        vec![1, 2, 3],
        vec![4, 5, 6],
        vec![7, 8, 9],
        vec![11, 12, 13],
        vec![14, 15, 16],
        vec![17, 18, 19],
    ];

    let cloud = points.into_eigen_cloud();
    for (id, row) in cloud.row_iter().enumerate() {
        assert_eq!(3, expected[id].len());
        assert_eq!(expected[id].len(), row.len());
        for (i, v) in row.iter().enumerate() {
            assert_eq!(expected[id][i] as f64, *v);
        }
    }
}

#[test]
fn test_add_cloud() {
    use super::*;

    let mut u = vec![
        PointXYZ::new(1.0, 2.0, 3.0),
        PointXYZ::new(4.0, 5.0, 6.0),
        PointXYZ::new(7.0, 8.0, 9.0),
    ];

    let v = vec![
        PointXYZ::new(11.0, 12.0, 13.0),
        PointXYZ::new(14.0, 15.0, 16.0),
        PointXYZ::new(17.0, 18.0, 19.0),
    ];

    add_cloud(&mut u, &v);

    let expected = vec![
        vec![1, 2, 3],
        vec![4, 5, 6],
        vec![7, 8, 9],
        vec![11, 12, 13],
        vec![14, 15, 16],
        vec![17, 18, 19],
    ];

    let cloud = u.into_eigen_cloud();
    for (id, row) in cloud.row_iter().enumerate() {
        assert_eq!(3, expected[id].len());
        assert_eq!(expected[id].len(), row.len());
        for (i, v) in row.iter().enumerate() {
            assert_eq!(expected[id][i] as f64, *v);
        }
    }
}

#[test]
fn test_estimate_plane() {
    use super::*;

    let params = Params::default();

    let mut workpp = PatchWorkpp::new(params);

    workpp.estimate_plane(&[
        PointXYZ::new(1.0, 2.0, 3.0),
        PointXYZ::new(4.0, 5.0, 6.0),
        PointXYZ::new(7.0, 8.0, 9.0),
        PointXYZ::new(11.0, 12.0, 13.0),
        PointXYZ::new(14.0, 15.0, 16.0),
        PointXYZ::new(17.0, 18.0, 19.0),
    ]);

    assert!(false)
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

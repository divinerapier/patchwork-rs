use nalgebra::OVector;

use crate::{IntoCloud, Params, Points, Ring, Zone};

pub struct Ground {
    pub(crate) timer: i64,
    pub(crate) time_taken: i64,
    pub(crate) update_flatness: [Vec<f64>; 4],
    pub(crate) update_elevation: [Vec<f64>; 4],
    pub d: f64,
    pub normal: OVector<f32, nalgebra::Const<3>>,
    pub(crate) singular_values: OVector<f32, nalgebra::Const<3>>,
    pub(crate) cov: nalgebra::Matrix3<f32>,

    pub(crate) pc_mean: OVector<f32, nalgebra::Const<3>>,

    pub(crate) min_ranges: Vec<f64>,
    pub(crate) sector_sizes: Vec<f64>,
    pub(crate) ring_sizes: Vec<f64>,

    // ConcentricZoneModel_
    pub(crate) concentric_zone_model: Vec<Zone>,

    pub(crate) ground_pc: Points,
    pub(crate) regionwise_ground: Points,
    pub(crate) regionwise_nonground: Points,
    pub(crate) cloud_ground: Points,
    pub(crate) cloud_nonground: Points,
    pub(crate) centers: Points,
    pub(crate) normals: Points,

    // from params
    pub(crate) elevation_thr: [f64; 4],
    pub(crate) flatness_thr: [f64; 4],
    pub(crate) sensor_height: f64,
}

impl Ground {
    pub fn new(params: &Params) -> Self {
        let min_range_z2 = (7.0 * params.min_range + params.max_range) / 8.0;
        let min_range_z3 = (3.0 * params.min_range + params.max_range) / 4.0;
        let min_range_z4 = (params.min_range + params.max_range) / 2.0;

        let min_ranges = vec![params.min_range, min_range_z2, min_range_z3, min_range_z4];

        let ring_sizes = vec![
            (min_range_z2 - params.min_range) / params.num_rings_each_zone[0] as f64,
            (min_range_z3 - min_range_z2) / params.num_rings_each_zone[1] as f64,
            (min_range_z4 - min_range_z3) / params.num_rings_each_zone[2] as f64,
            (params.max_range - min_range_z4) / params.num_rings_each_zone[3] as f64,
        ];

        let sector_sizes = vec![
            2.0 * std::f64::consts::PI / params.num_sectors_each_zone[0] as f64,
            2.0 * std::f64::consts::PI / params.num_sectors_each_zone[1] as f64,
            2.0 * std::f64::consts::PI / params.num_sectors_each_zone[2] as f64,
            2.0 * std::f64::consts::PI / params.num_sectors_each_zone[3] as f64,
        ];

        let mut concentric_zone_model = vec![];

        for k in 0..params.num_zones as usize {
            let empty_ring = Ring::from(vec![
                Points::from(vec![]);
                params.num_sectors_each_zone[k] as usize
            ]);
            let mut z = Zone::default();

            for _ in 0..params.num_rings_each_zone[k] as usize {
                z.push(empty_ring.clone());
            }

            concentric_zone_model.push(z);
        }

        Self {
            min_ranges,
            ring_sizes,
            sector_sizes,
            concentric_zone_model,

            timer: 0,
            time_taken: 0,
            update_flatness: Default::default(),
            update_elevation: Default::default(),
            d: 0.0,
            normal: nalgebra::vector![0.0, 0.0, 0.0],
            singular_values: nalgebra::vector![0.0, 0.0, 0.0],
            cov: nalgebra::Matrix3::zeros(),
            pc_mean: nalgebra::vector![0.0, 0.0, 0.0],
            ground_pc: Points::from(vec![]),
            regionwise_ground: Points::from(vec![]),
            regionwise_nonground: Points::from(vec![]),
            cloud_ground: Points::from(vec![]),
            cloud_nonground: Points::from(vec![]),
            centers: Points::from(vec![]),
            normals: Points::from(vec![]),

            // from params
            /// threshold of elevation for each ring using in GLE. Those values are updated adaptively.
            elevation_thr: [0.0, 0.0, 0.0, 0.0],

            /// threshold of flatness for each ring using in GLE. Those values are updated adaptively.
            flatness_thr: [0.0, 0.0, 0.0, 0.0],

            sensor_height: 1.723,
        }
    }

    pub fn get_ground(&self) -> nalgebra::MatrixXx3<f32> {
        self.cloud_ground.into_cloud()
    }

    pub fn get_nonground(&self) -> nalgebra::MatrixXx3<f32> {
        self.cloud_nonground.into_cloud()
    }

    pub fn get_centers(&self) -> nalgebra::MatrixXx3<f32> {
        self.centers.into_cloud()
    }

    pub fn get_normals(&self) -> nalgebra::MatrixXx3<f32> {
        self.normals.into_cloud()
    }
}

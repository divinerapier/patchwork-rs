#[derive(Debug)]
pub struct Params {
    pub(crate) verbose: bool,
    pub(crate) enable_RNR: bool,
    pub(crate) enable_RVPF: bool,
    pub(crate) enable_TGR: bool,

    pub(crate) num_iter: i32,
    pub(crate) num_lpr: i32,
    pub(crate) num_min_pts: usize,
    pub(crate) num_zones: usize,
    pub(crate) num_rings_of_interest: i32,

    pub(crate) RNR_ver_angle_thr: f64,
    pub(crate) RNR_intensity_thr: f64,

    pub(crate) sensor_height: f64,
    pub(crate) th_seeds: f64,
    pub(crate) th_dist: f64,
    pub(crate) th_seeds_v: f64,
    pub(crate) th_dist_v: f64,
    pub(crate) max_range: f64,
    pub(crate) min_range: f64,
    pub(crate) uprightness_thr: f64,
    pub(crate) adaptive_seed_selection_margin: f64,
    pub(crate) intensity_thr: f64,

    pub(crate) num_sectors_each_zone: Vec<usize>,
    pub(crate) num_rings_each_zone: Vec<usize>,

    pub(crate) max_flatness_storage: i32,
    pub(crate) max_elevation_storage: i32,

    pub(crate) elevation_thr: [f64; 4],
    pub(crate) flatness_thr: [f64; 4],
}

impl Default for Params {
    fn default() -> Self {
        Self {
            verbose: false,
            enable_RNR: true,
            enable_RVPF: true,
            enable_TGR: true,

            /// Number of iterations for ground plane estimation using PCA.
            num_iter: 3,

            /// Maximum number of points to be selected as lowest points representative.
            num_lpr: 20,

            /// Minimum number of points to be estimated as ground plane in each patch.
            num_min_pts: 10,

            /// Setting of Concentric Zone Model(CZM)
            num_zones: 4,

            /// Number of rings to be checked with elevation and flatness values.
            num_rings_of_interest: 4,

            /// Noise points vertical angle threshold. Downward rays of LiDAR are more likely to generate severe noise points.
            RNR_ver_angle_thr: -15.0,

            /// Noise points intensity threshold. The reflected points have relatively small intensity than others.
            RNR_intensity_thr: 0.2,

            sensor_height: 1.723,

            /// threshold for lowest point representatives using in initial seeds selection of ground points.
            th_seeds: 0.125,

            /// threshold for thickenss of ground.
            th_dist: 0.125,

            /// threshold for lowest point representatives using in initial seeds selection of vertical structural points.
            th_seeds_v: 0.25,

            /// threshold for thickenss of vertical structure.
            th_dist_v: 0.1,

            /// max_range of ground estimation area
            max_range: 80.0,

            /// min_range of ground estimation area
            min_range: 2.7,

            /// threshold of uprightness using in Ground Likelihood Estimation(GLE). Please refer paper for more information about GLE.
            uprightness_thr: 0.707,

            /// parameter using in initial seeds selection
            adaptive_seed_selection_margin: -1.2,

            /// The minimum intensity of ground points. The points with intensity smaller than this value are not considered as ground points.
            intensity_thr: 0.0,

            /// Setting of Concentric Zone Model(CZM)
            num_sectors_each_zone: vec![16, 32, 54, 32],
            /// Setting of Concentric Zone Model(CZM)
            num_rings_each_zone: vec![2, 4, 4, 4],

            /// The maximum number of flatness storage
            max_flatness_storage: 1000,

            /// The maximum number of elevation storage
            max_elevation_storage: 1000,

            /// threshold of elevation for each ring using in GLE. Those values are updated adaptively.
            elevation_thr: [0.0, 0.0, 0.0, 0.0],

            /// threshold of flatness for each ring using in GLE. Those values are updated adaptively.
            flatness_thr: [0.0, 0.0, 0.0, 0.0],
        }
    }
}

use std::{f64::consts::PI, println, time::Instant, unreachable};

use nalgebra::{Dim, MatrixXx3, OVector, RawStorage, Vector3};

mod tests;

#[derive(Debug, Clone, Copy)]
pub struct PointXYZ {
    x: f64,
    y: f64,
    z: f64,
}

impl PointXYZ {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }
}

pub struct RevertCandidate {
    concentric_idx: i32,
    sector_idx: i32,
    ground_flatness: f64,
    line_variable: f64,
    pc_mean: OVector<f64, nalgebra::Const<3>>,
    regionwise_ground: Vec<PointXYZ>,
}

impl RevertCandidate {
    pub fn new(
        concentric_idx: i32,
        sector_idx: i32,
        ground_flatness: f64,
        line_variable: f64,
        pc_mean: OVector<f64, nalgebra::Const<3>>,
        regionwise_ground: Vec<PointXYZ>,
    ) -> Self {
        Self {
            concentric_idx,
            sector_idx,
            ground_flatness,
            line_variable,
            pc_mean,
            regionwise_ground,
        }
    }
}

pub struct Params {
    verbose: bool,
    enable_RNR: bool,
    enable_RVPF: bool,
    enable_TGR: bool,

    num_iter: i32,
    num_lpr: i32,
    num_min_pts: usize,
    num_zones: usize,
    num_rings_of_interest: i32,

    RNR_ver_angle_thr: f64,
    RNR_intensity_thr: f64,

    sensor_height: f64,
    th_seeds: f64,
    th_dist: f64,
    th_seeds_v: f64,
    th_dist_v: f64,
    max_range: f64,
    min_range: f64,
    uprightness_thr: f64,
    adaptive_seed_selection_margin: f64,
    intensity_thr: f64,

    num_sectors_each_zone: Vec<usize>,
    num_rings_each_zone: Vec<usize>,

    max_flatness_storage: i32,
    max_elevation_storage: i32,

    elevation_thr: Vec<f64>,
    flatness_thr: Vec<f64>,
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
            elevation_thr: vec![0.0, 0.0, 0.0, 0.0],

            /// threshold of flatness for each ring using in GLE. Those values are updated adaptively.
            flatness_thr: vec![0.0, 0.0, 0.0, 0.0],
        }
    }
}

type Ring = Vec<Vec<PointXYZ>>;
type Zone = Vec<Ring>;

pub struct PatchWorkpp {
    params: Params,
    timer: i64,
    time_taken: i64,
    update_flatness: [Vec<f64>; 4],
    update_elevation: [Vec<f64>; 4],
    d: f64,
    normal: OVector<f64, nalgebra::Const<3>>,
    singular_values: OVector<f64, nalgebra::Const<3>>,
    cov: nalgebra::Matrix3<f64>,
    // 列向量
    pc_mean: OVector<f64, nalgebra::Const<3>>,
    min_ranges: Vec<f64>,
    sector_sizes: Vec<f64>,
    ring_sizes: Vec<f64>,

    // ConcentricZoneModel_
    concentric_zone_model: Vec<Zone>,

    ground_pc: Vec<PointXYZ>,
    // non_ground_pc: Vec<PointXYZ>,
    regionwise_ground: Vec<PointXYZ>,
    regionwise_nonground: Vec<PointXYZ>,
    cloud_ground: Vec<PointXYZ>,
    cloud_nonground: Vec<PointXYZ>,
    centers: Vec<PointXYZ>,
    normals: Vec<PointXYZ>,
}

impl PatchWorkpp {
    pub fn new(params: Params) -> Self {
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

        println!("ring_sizes: {:?}", ring_sizes);

        let sector_sizes = vec![
            2.0 * std::f64::consts::PI / params.num_sectors_each_zone[0] as f64,
            2.0 * std::f64::consts::PI / params.num_sectors_each_zone[1] as f64,
            2.0 * std::f64::consts::PI / params.num_sectors_each_zone[2] as f64,
            2.0 * std::f64::consts::PI / params.num_sectors_each_zone[3] as f64,
        ];

        println!("sector_sizes: {:?}", sector_sizes);

        let mut concentric_zone_model = vec![];

        for k in 0..params.num_zones as usize {
            let empty_ring = vec![vec![]; params.num_sectors_each_zone[k] as usize]; // Ring::with_capacity(params.num_sectors_each_zone[k] as usize);
            let mut z = Zone::default();

            for _ in 0..params.num_rings_each_zone[k] as usize {
                z.push(empty_ring.clone());
            }

            concentric_zone_model.push(z);
        }

        // println!("concentric_zone_model: {:?}", concentric_zone_model);
        // for (i, zone) in concentric_zone_model.iter().enumerate() {
        //     for (j, ring) in zone.iter().enumerate() {
        //         for (k, points) in ring.iter().enumerate() {
        //             println!(
        //                 "concentric_zone_model[{:02?}][{:02?}][{:02?}] = {}",
        //                 i,
        //                 j,
        //                 k,
        //                 points.len()
        //             );
        //         }
        //     }
        // }

        PatchWorkpp {
            params,
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
            ground_pc: vec![],
            // non_ground_pc: vec![],
            regionwise_ground: vec![],
            regionwise_nonground: vec![],
            cloud_ground: vec![],
            cloud_nonground: vec![],
            centers: vec![],
            normals: vec![],
        }
    }
}

pub trait IntoEigenCloud {
    fn into_eigen_cloud(self) -> nalgebra::MatrixXx3<f64>;
}

impl IntoEigenCloud for Vec<PointXYZ> {
    fn into_eigen_cloud(self) -> nalgebra::MatrixXx3<f64> {
        let mut cloud = nalgebra::MatrixXx3::zeros(self.len());

        for (i, p) in self.iter().enumerate() {
            cloud[(i, 0)] = p.x;
            cloud[(i, 1)] = p.y;
            cloud[(i, 2)] = p.z;
        }

        cloud
    }
}

pub fn add_cloud(cloud: &mut Vec<PointXYZ>, points: &[PointXYZ]) {
    cloud.extend(points);
}

pub fn compare_point_z(a: &PointXYZ, b: &PointXYZ) -> std::cmp::Ordering {
    a.z.partial_cmp(&b.z).unwrap_or(std::cmp::Ordering::Equal)
}

impl PatchWorkpp {
    pub fn flush_patches(&mut self) {
        let czm = &mut self.concentric_zone_model;
        for k in 0..self.params.num_zones {
            for i in 0..self.params.num_rings_each_zone[k] {
                for j in 0..self.params.num_sectors_each_zone[k] {
                    czm[k][i][j].clear();
                }
            }
        }
    }

    pub fn estimate_plane(&mut self, ground: &[PointXYZ]) {
        if ground.is_empty() {
            return;
        }
        let mut eigen_ground = nalgebra::MatrixXx3::zeros(ground.len());

        for (j, p) in ground.iter().enumerate() {
            eigen_ground[(j, 0)] = p.x;
            eigen_ground[(j, 1)] = p.y;
            eigen_ground[(j, 2)] = p.z;
        }

        // // eigen_ground.colwise().mean() 计算每一列的平均值 (mean)，假设原矩阵为 M x N，则计算结果为 1 x N 维向量
        // // eigen_ground.rowwise() - eigen_ground.colwise().mean() 每一行与后面的向量计算减法
        // Eigen::MatrixX3f centered = eigen_ground.rowwise() - eigen_ground.colwise().mean();
        // Eigen::MatrixX3f cov = (centered.adjoint() * centered) / double(eigen_ground.rows() - 1);
        //
        let mean = eigen_ground.row_mean();
        debug_matrix("mean", &mean);
        {
            self.pc_mean[0] = mean[0];
            self.pc_mean[1] = mean[1];
            self.pc_mean[2] = mean[2];
            debug_matrix("pc mean", &self.pc_mean);
        }

        let centered = eigen_ground
            .row_iter()
            .map(|row| row - &mean)
            .collect::<Vec<_>>();

        let centered = MatrixXx3::from_rows(centered.as_slice());
        debug_matrix("centered", &centered);

        let cov = (centered.adjoint() * &centered) / (centered.nrows() as f64 - 1.0);
        debug_matrix("cov", &cov);

        // https://github.com/dimforge/nalgebra/issues/1072#issuecomment-1029808465
        // let mut svd = cov.try_svd(true, true, 4.0 * f64::EPSILON, 0).unwrap();
        let mut svd = nalgebra::SVD::new(cov, true, true);
        self.singular_values = svd.singular_values;
        debug_matrix("singular_values", &self.singular_values);

        let u = svd.u.take().unwrap();
        debug_matrix("svd.u", &u);

        // v_t 就是转置过的矩阵，就是公式中的 A = U∑V^T 中的 V^T
        // 其中 ∑ 是一个对角矩阵，对角线上的元素就是 svd.singular_values
        let v_t = svd.v_t.take().unwrap();
        debug_matrix("svd.v_t", &v_t);

        debug_matrix(
            "xx",
            &(u * nalgebra::matrix![
                self.singular_values [0], 0.0, 0.0;
                0.0, self.singular_values [1], 0.0;
                0.0, 0.0, self.singular_values [2];
            ] * v_t),
        );

        // let u = svd.v_t.take().unwrap().transpose();
        // debug_matrix("svd.u", &u);

        let normal = u.column(2).to_owned();
        self.normal = Vector3::new(normal[0], normal[1], normal[2]);
        debug_matrix("normal", &self.normal);

        if self.normal[2] < 0.0 {
            self.normal = -self.normal;
        }
        debug_matrix("normal", &self.normal);

        let seeds_mean = self.pc_mean;
        debug_matrix("seeds_mean", &seeds_mean);

        let d = self.normal.transpose() * seeds_mean;
        debug_matrix("d", &d);

        self.d = d[0];
        println!("self.d: {:?}\n", self.d);
    }

    pub fn extract_initial_seeds_with_th_seed(
        params: &Params,
        zone_index: usize,
        p_sorted: &[PointXYZ],
        init_seeds: &mut Vec<PointXYZ>,
        th_seeds: f64,
    ) {
        init_seeds.clear();

        let mut sum = 0.0;
        let mut cnt = 0;
        let mut init_idx = 0;

        if zone_index == 0 {
            for i in 0..p_sorted.len() {
                if p_sorted[i].z < params.adaptive_seed_selection_margin * params.sensor_height {
                    init_idx += 1;
                } else {
                    break;
                }
            }
        }

        // Calculate the mean height value.
        for i in init_idx..p_sorted.len() {
            if cnt >= params.num_lpr {
                break;
            }
            sum += p_sorted[i].z;
            cnt += 1;
        }

        let lpr_height = if cnt != 0 { sum / (cnt as f64) } else { 0.0 };

        for i in 0..p_sorted.len() {
            if p_sorted[i].z < lpr_height + th_seeds {
                init_seeds.push(p_sorted[i].clone());
            }
        }
    }

    pub fn estimate_ground<
        R: Dim,
        C: Dim,
        S: std::fmt::Debug + RawStorage<f64, R, C> + nalgebra::RawStorageMut<f64, R, C>,
    >(
        &mut self,
        cloud_in: &mut nalgebra::Matrix<f64, R, C, S>,
    ) {
        self.cloud_ground.clear();
        self.cloud_nonground.clear();

        if self.params.verbose {
            // TODO: tracing
        }

        // let start = Instant::now();

        if self.params.enable_RNR {
            self.reflected_noise_removal(cloud_in);
        }

        // let t1 = Instant::now();

        self.flush_patches();

        // let t1_1 = Instant::now();

        self.pc2czm(cloud_in);

        // let t2 = Instant::now();

        let mut concentric_idx = 0;

        self.centers.clear();
        self.normals.clear();

        // let t_flush = t1_1.duration_since(t1);
        // let t_czm = t2.duration_since(t1_1);

        // let mut t_sort = 0.0;
        // let mut t_pca = 0.0;
        // let mut t_gle = 0.0;
        // let mut t_revert = 0.0;
        // let mut t_update = 0.0;

        let mut candidates = Vec::<RevertCandidate>::new();
        let mut ringwise_flatness = Vec::<f64>::new();

        let num_zones = self.params.num_zones as usize;

        for zone_idx in 0..num_zones {
            // let zone = &mut self.concentric_zone_model[zone_idx];

            for ring_idx in 0..self.params.num_rings_each_zone[zone_idx] {
                for sector_idx in 0..self.params.num_sectors_each_zone[zone_idx] {
                    let zone = &mut self.concentric_zone_model[zone_idx];
                    if zone[ring_idx][sector_idx].len() < self.params.num_min_pts {
                        add_cloud(&mut self.cloud_nonground, &zone[ring_idx][sector_idx]);
                        continue;
                    }
                    // --------- region-wise sorting (faster than global sorting method) ---------------- //
                    let zone = &mut self.concentric_zone_model[zone_idx];
                    // let t_before_sort = Instant::now();
                    zone[ring_idx][sector_idx].sort_by(compare_point_z);
                    // let t_after_sort = Instant::now();

                    // t_sort = t_sort + t_after_sort.duration_since(t_before_sort).as_secs_f64();

                    // --------- PCA --------- //
                    // let t_before_pca = Instant::now();
                    self.extract_piecewiseground(zone_idx, ring_idx, sector_idx);
                    // let t_after_pca = Instant::now();
                    // t_pca = t_pca + t_after_pca.duration_since(t_before_pca).as_secs_f64();
                    self.centers.push(PointXYZ::new(
                        self.pc_mean[0],
                        self.pc_mean[1],
                        self.pc_mean[2],
                    ));
                    self.normals.push(PointXYZ::new(
                        self.normal[0],
                        self.normal[1],
                        self.normal[2],
                    ));

                    // --------- GLE --------- //
                    // let t_before_gle = Instant::now();
                    let ground_uprightness = self.normal[2];
                    let ground_elevation = self.pc_mean[2];
                    let ground_flatness = self.singular_values[2];
                    let line_variable = if self.singular_values[1] != 0.0 {
                        self.singular_values[0] / self.singular_values[1]
                    } else {
                        // https://en.cppreference.com/w/cpp/types/climits
                        std::f64::MAX
                    };
                    let mut heading = 0.0;
                    for i in 0..3 {
                        heading = heading + (self.pc_mean[i] * self.normal[i]);
                    }
                    // About 'is_heading_outside' condition, heading should be smaller than 0 theoretically.
                    // ( Imagine the geometric relationship between the surface normal vector on the ground plane and
                    //     the vector connecting the sensor origin and the mean point of the ground plane )
                    //
                    // However, when the patch is far awaw from the sensor origin,
                    // heading could be larger than 0 even if it's ground due to lack of amount of ground plane points.
                    //
                    // Therefore, we only check this value when concentric_idx < num_rings_of_interest ( near condition )
                    let is_upright = ground_uprightness > self.params.uprightness_thr;
                    let is_not_elevated =
                        ground_elevation < self.params.elevation_thr[concentric_idx];
                    let is_flat = ground_flatness < self.params.flatness_thr[concentric_idx];
                    let is_near_zone = concentric_idx < self.params.num_rings_of_interest as usize;
                    let is_heading_outside = heading < 0.0;

                    // Store the elevation & flatness variables
                    // for A-GLE (Adaptive Ground Likelihood Estimation)
                    // and TGR (Temporal Ground Revert). More information in the paper Patchwork++.

                    if is_upright && is_not_elevated && is_near_zone {
                        self.update_elevation[concentric_idx].push(ground_elevation);
                        self.update_flatness[concentric_idx].push(ground_flatness);

                        ringwise_flatness.push(ground_flatness);
                    }

                    // Ground estimation based on conditions
                    if !is_upright {
                        add_cloud(&mut self.cloud_nonground, &self.regionwise_ground);
                    } else if !is_near_zone {
                        add_cloud(&mut self.cloud_ground, &self.regionwise_ground);
                    } else if !is_heading_outside {
                        add_cloud(&mut self.cloud_nonground, &self.regionwise_ground);
                    } else if is_not_elevated || is_flat {
                        add_cloud(&mut self.cloud_ground, &self.regionwise_ground);
                    } else {
                        // patchwork::RevertCandidate candidate(concentric_idx, sector_idx, ground_flatness, line_variable,
                        //                                      pc_mean_, regionwise_ground_);
                        // candidates.push_back(candidate);
                        candidates.push(RevertCandidate::new(
                            concentric_idx as i32,
                            sector_idx as i32,
                            ground_flatness,
                            line_variable,
                            self.pc_mean.clone(),
                            self.regionwise_ground.clone(),
                        ));
                    }

                    add_cloud(&mut self.cloud_nonground, &self.regionwise_nonground);

                    // let t_after_gle = Instant::now();
                    // t_gle = t_gle + t_after_gle.duration_since(t_before_gle).as_secs_f64();
                }

                // --------- Revert --------- //
                // let t_before_revert = Instant::now();

                if !candidates.is_empty() {
                    if self.params.enable_TGR {
                        self.temporal_ground_revert(&ringwise_flatness, &candidates, concentric_idx)
                    } else {
                        for candidate in &candidates {
                            add_cloud(&mut self.cloud_nonground, &candidate.regionwise_ground);
                        }
                    }
                    candidates.clear();
                    ringwise_flatness.clear();
                }

                // let t_after_revert = Instant::now();
                // t_revert =
                //     t_revert + t_after_revert.duration_since(t_before_revert).as_secs_f64();

                concentric_idx += 1;
            }
        }
        // let t_before_update = Instant::now();
        self.update_elevation_thr();
        self.update_flatness_thr();
        // let t_after_update = Instant::now();
        // t_update = t_after_update.duration_since(t_before_update).as_secs_f64();

        // let end = Instant::now();
        // let t_total = end.duration_since(start).as_secs_f64();
    }

    fn update_elevation_thr(&mut self) {
        for i in 0..self.params.num_rings_of_interest as usize {
            if self.update_elevation[i].is_empty() {
                continue;
            }
            let (update_mean, update_stdev) = self.calc_mean_stdev(&self.update_elevation[i]);
            if i == 0 {
                self.params.elevation_thr[i] = update_mean + 3.0 * update_stdev;
                self.params.sensor_height = -update_mean;
            } else {
                self.params.elevation_thr[i] = update_mean + 2.0 * update_stdev;
            }

            let exceed_num =
                self.update_elevation[i].len() as i32 - self.params.max_elevation_storage;
            if exceed_num > 0 {
                self.update_elevation[i].drain(0..exceed_num as usize);
            }
        }
    }

    fn update_flatness_thr(&mut self) {
        for i in 0..self.params.num_rings_of_interest as usize {
            if self.update_flatness[i].is_empty() {
                break;
            }
            if self.update_flatness[i].len() < 1 {
                break;
            }

            let (update_mean, update_stdev) = self.calc_mean_stdev(&self.update_flatness[i]);
            self.params.flatness_thr[i] = update_mean + update_stdev;

            let exceed_num =
                self.update_flatness[i].len() as i32 - self.params.max_flatness_storage;
            if exceed_num > 0 {
                self.update_flatness[i].drain(0..exceed_num as usize);
            }
        }
    }

    fn temporal_ground_revert(
        &mut self,
        ring_flatness: &[f64],
        candidates: &[RevertCandidate],
        concentric_idx: usize,
    ) {
        let (mean_flatness, stdev_flatness) = self.calc_mean_stdev(&ring_flatness);

        for candidate in candidates {
            let mu_flatness = mean_flatness + 1.5 * stdev_flatness;
            let prob_flatness = if candidate.regionwise_ground.len() > 1500
                && candidate.ground_flatness < self.params.th_dist * self.params.th_dist
            {
                1.0
            } else {
                1.0 / (1.0
                    + ((candidate.ground_flatness - mu_flatness) / (mu_flatness / 10.0)).exp())
            };

            let prob_line = if candidate.line_variable > 8.0 {
                0.0
            } else {
                1.0
            };

            let revert = prob_line * prob_flatness > 0.5;

            if concentric_idx < self.params.num_rings_of_interest as usize {
                if revert {
                    add_cloud(&mut self.cloud_ground, &candidate.regionwise_ground);
                } else {
                    add_cloud(&mut self.cloud_nonground, &candidate.regionwise_ground);
                }
            }
        }
    }

    fn calc_mean_stdev(&self, v: &[f64]) -> (f64, f64) {
        if v.len() <= 1 {
            return (0.0, 0.0);
        }

        let mean = v.iter().sum::<f64>() / v.len() as f64;

        let stdev = v.iter().map(|x| (*x - mean) * (*x + mean)).sum::<f64>() / (v.len() - 1) as f64;

        let stdev = stdev.sqrt();

        (mean, stdev)
    }

    fn reflected_noise_removal<
        R: Dim,
        C: Dim,
        S: std::fmt::Debug + RawStorage<f64, R, C> + nalgebra::RawStorageMut<f64, R, C>,
    >(
        &mut self,
        cloud_in: &mut nalgebra::Matrix<f64, R, C, S>,
    ) {
        if cloud_in.row(0).len() < 4 {
            return;
        }

        let mut cnt = 0;

        for mut row in cloud_in.row_iter_mut() {
            // double r = sqrt(
            //     cloud_in.row(i)(0) * cloud_in.row(i)(0) + cloud_in.row(i)(1) * cloud_in.row(i)(1));
            let a = row[0];
            let b = row[1];
            let r = (a * a + b * b).sqrt();
            let z = row[2];
            let ver_angle_in_deg = z.atan2(r).to_degrees();
            // println!("ver_angle_in_deg: {}", ver_angle_in_deg);
            if ver_angle_in_deg < self.params.RNR_ver_angle_thr
                && z < -self.params.sensor_height - 0.8
                && row[3] < self.params.RNR_intensity_thr
            {
                //        cloud_nonground_.push_back(PointXYZ(cloud_in.row(i)(0), cloud_in.row(i)(1), cloud_in.row(i)(2)));
                // cloud_in.row(i)(2) = std::numeric_limits<float>::min();
                // cnt++;
                self.cloud_ground
                    .push(PointXYZ::new(row[0], row[1], row[2]));
                row[2] = f64::MIN;
                cnt += 1;
            }
        }
        if self.params.verbose {
            // TODO: tracing
        }
    }

    fn pc2czm<R: Dim, C: Dim, S: std::fmt::Debug + RawStorage<f64, R, C>>(
        &mut self,
        src: &nalgebra::Matrix<f64, R, C, S>,
        // cmz: &mut [Zone],
    ) {
        let max_range = self.params.max_range;
        let min_range = self.params.min_range;
        let min_ranges = [
            self.min_ranges[0],
            self.min_ranges[1],
            self.min_ranges[2],
            self.min_ranges[3],
        ];
        let num_ring = [
            self.params.num_rings_each_zone[0],
            self.params.num_rings_each_zone[1],
            self.params.num_rings_each_zone[2],
            self.params.num_rings_each_zone[3],
        ];
        let num_sector = [
            self.params.num_sectors_each_zone[0],
            self.params.num_sectors_each_zone[1],
            self.params.num_sectors_each_zone[2],
            self.params.num_sectors_each_zone[3],
        ];

        // println!("max_range: {:?}", max_range);
        // println!("min_range: {:?}", min_range);
        // println!("min_ranges: {:?}", min_ranges);
        // println!("num_ring: {:?}", num_ring);
        // println!("num_sector: {:?}", num_sector);

        let (rows, _) = src.row_iter().enumerate().last().unwrap();
        let rows = rows + 1;
        // println!("rows: {:?}", rows);

        for (i, row) in src.row_iter().enumerate() {
            let x = row[0];
            let y = row[1];
            let z = row[2];

            let r = xy2radius(x, y);
            if (r <= max_range) && (r > min_range) {
                let theta = xy2theta(x, y);

                let index = if r < min_ranges[1] {
                    // In First rings
                    0
                } else if r < min_ranges[2] {
                    1
                } else if r < min_ranges[3] {
                    2
                } else {
                    // Far!
                    3
                };

                let a = (r - min_ranges[index]) / self.ring_sizes[index];
                let ring_idx = (a as usize).min(num_ring[index] - 1);
                let b = theta / self.sector_sizes[index];
                let sector_idx = (b as usize).min(num_sector[index] - 1);
                // println!(
                //     "{}",
                //     serde_json::to_string(&serde_json::json!({
                //         "i": i,
                //         "index": index,
                //         "x": x,
                //         "y": y,
                //         "z": z,
                //         "r": r,
                //         "theta": theta,
                //         "ring_idx": ring_idx,
                //         "sector_idx": sector_idx,
                //     }))
                //     .unwrap()
                // );
                // println!(
                //     "index: {}, ring_idx: {}, sector_idx: {}",
                //     index, ring_idx, sector_idx
                // );
                self.concentric_zone_model[index][ring_idx][sector_idx]
                    .push(PointXYZ::new(x, y, z));
            } else {
                self.cloud_nonground.push(PointXYZ::new(x, y, z));
            }
        }
    }

    fn extract_piecewiseground(
        &mut self,
        zone_index: usize,
        ring_index: usize,
        sector_index: usize,
    ) {
        if !self.ground_pc.is_empty() {
            self.ground_pc.clear();
        }

        let dst = &mut self.regionwise_ground;
        if !dst.is_empty() {
            dst.clear();
        }

        let non_ground_dst = &mut self.regionwise_nonground;
        if !non_ground_dst.is_empty() {
            non_ground_dst.clear();
        }

        // 1. Region-wise Vertical Plane Fitting (R-VPF)
        // : removes potential vertical plane under the ground plane

        let mut src_wo_verticals =
            self.concentric_zone_model[zone_index][ring_index][sector_index].clone();

        if self.params.enable_RVPF {
            for _i in 0..self.params.num_iter {
                Self::extract_initial_seeds_with_th_seed(
                    &self.params,
                    zone_index,
                    &src_wo_verticals,
                    &mut self.ground_pc,
                    self.params.th_seeds_v,
                );
                let ground_pc = self.ground_pc.clone();
                self.estimate_plane(&ground_pc);

                if zone_index == 0 && self.normal[2] < self.params.uprightness_thr {
                    let src_tmp = &src_wo_verticals;
                    let iter = src_tmp
                        .into_iter()
                        .map(|point| (point, calc_point_to_plane_d(point, &self.normal, self.d)))
                        .into_split_filter(|(_, distance)| distance.abs() < self.params.th_dist_v);

                    let mut new_verticals = vec![];
                    for item in iter {
                        match item {
                            (Some((point, _)), None) => {
                                self.regionwise_nonground.push(*point);
                            }
                            (None, Some((point, _))) => {
                                new_verticals.push(*point);
                            }
                            _ => unreachable!(),
                        }
                    }
                    src_wo_verticals = new_verticals;
                } else {
                    break;
                }
            }
        }

        Self::extract_initial_seeds_with_th_seed(
            &self.params,
            zone_index,
            &src_wo_verticals,
            &mut self.ground_pc,
            self.params.th_seeds,
        );

        self.estimate_plane(&self.ground_pc.clone());

        for i in 0..self.params.num_iter {
            self.ground_pc.clear();

            for point in &src_wo_verticals {
                let distance = calc_point_to_plane_d(point, &self.normal, self.d);

                if i < self.params.num_iter - 1 {
                    if distance < self.params.th_dist {
                        self.ground_pc.push(*point);
                    }
                } else {
                    if distance < self.params.th_dist {
                        self.regionwise_ground.push(*point);
                    } else {
                        self.regionwise_nonground.push(*point);
                    }
                }
            }

            if i < self.params.num_iter - 1 {
                self.estimate_plane(&self.ground_pc.clone());
            } else {
                self.estimate_plane(&self.regionwise_ground.clone());
            }
        }
    }
}

fn calc_point_to_plane_d(
    point: &PointXYZ,
    normal: &OVector<f64, nalgebra::Const<3>>,
    d: f64,
) -> f64 {
    normal[0] * point.x + normal[1] * point.y + normal[2] * point.z + d
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

fn xy2radius(x: f64, y: f64) -> f64 {
    (x * x + y * y).sqrt()
}

fn xy2theta(x: f64, y: f64) -> f64 {
    let angle = y.atan2(x);
    if angle > 0. {
        angle
    } else {
        angle + 2.0 * PI
    }
}

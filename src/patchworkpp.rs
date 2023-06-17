use nalgebra::{Dim, MatrixXx3, OVector, RawStorage, Vector3};

use crate::{
    cloud::IntoCloud,
    utils::{debug_matrix, xy2radius, xy2theta, IntoSplitFilter},
    Params, Point3D, Points, Ring, Zone,
};

#[derive(Debug)]
pub struct PatchWorkpp {
    pub(crate) params: Params,
    timer: i64,
    time_taken: i64,
    update_flatness: [Vec<f64>; 4],
    update_elevation: [Vec<f64>; 4],
    pub d: f64,
    pub normal: OVector<f32, nalgebra::Const<3>>,
    singular_values: OVector<f32, nalgebra::Const<3>>,
    cov: nalgebra::Matrix3<f32>,

    pc_mean: OVector<f32, nalgebra::Const<3>>,

    min_ranges: Vec<f64>,
    sector_sizes: Vec<f64>,
    ring_sizes: Vec<f64>,

    // ConcentricZoneModel_
    concentric_zone_model: Vec<Zone>,

    ground_pc: Points,
    // non_ground_pc: Vec<PointXYZ>,
    regionwise_ground: Points,
    regionwise_nonground: Points,
    cloud_ground: Points,
    cloud_nonground: Points,
    centers: Points,
    normals: Points,
}

pub enum EstimatePlaneTarget {
    GroundPC,
    RegionwiseGround,
    Other(Points),
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
            let empty_ring = Ring::from(vec![
                Points::from(vec![]);
                params.num_sectors_each_zone[k] as usize
            ]); // Ring::with_capacity(params.num_sectors_each_zone[k] as usize);
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
            ground_pc: Points::from(vec![]),
            // non_ground_pc: vec![],
            regionwise_ground: Points::from(vec![]),
            regionwise_nonground: Points::from(vec![]),
            cloud_ground: Points::from(vec![]),
            cloud_nonground: Points::from(vec![]),
            centers: Points::from(vec![]),
            normals: Points::from(vec![]),
        }
    }
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

    pub fn estimate_plane(&mut self, target: EstimatePlaneTarget) {
        let ground = match target {
            EstimatePlaneTarget::GroundPC => &self.ground_pc,
            EstimatePlaneTarget::RegionwiseGround => &self.regionwise_ground,
            EstimatePlaneTarget::Other(ref points) => points,
        };

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
        let mean = eigen_ground.row_mean();

        // debug_matrix("mean", &mean);
        self.pc_mean[0] = mean[0];
        self.pc_mean[1] = mean[1];
        self.pc_mean[2] = mean[2];
        // debug_matrix("pc mean", &self.pc_mean);

        let centered = eigen_ground
            .row_iter()
            .map(|row| row - &mean)
            .collect::<Vec<_>>();

        let centered = MatrixXx3::from_rows(centered.as_slice());
        // debug_matrix("centered", &centered);

        let cov = (centered.adjoint() * &centered) / (centered.nrows() as f32 - 1.0);
        // debug_matrix("cov", &cov);

        // https://github.com/dimforge/nalgebra/issues/1072#issuecomment-1029808465
        // let mut svd = cov.try_svd(true, true, 4.0 * f64::EPSILON, 0).unwrap();
        let mut svd = nalgebra::SVD::new(cov, true, true);
        self.singular_values = svd.singular_values;
        // debug_matrix("singular_values", &self.singular_values);

        let u = svd.u.take().unwrap();
        // debug_matrix("svd.u", &u);

        // v_t 就是转置过的矩阵，就是公式中的 A = U∑V^T 中的 V^T
        // 其中 ∑ 是一个对角矩阵，对角线上的元素就是 svd.singular_values
        let v_t = svd.v_t.take().unwrap();
        // debug_matrix("svd.v_t", &v_t);

        let normal = u.column(2);
        self.normal = Vector3::new(normal[0], normal[1], normal[2]);
        // debug_matrix("normal", &self.normal);

        if self.normal[2] < 0.0 {
            self.normal = -self.normal;
        }
        debug_matrix("normal", &self.normal);

        let seeds_mean = self.pc_mean;
        // debug_matrix("seeds_mean", &seeds_mean);

        let d = self.normal.transpose() * seeds_mean;
        // debug_matrix("d", &d);

        self.d = -d[0] as f64;
        // println!("self.d: {:?}\n", self.d);
    }

    pub fn extract_initial_seeds_with_th_seed(
        params: &Params,
        zone_index: usize,
        p_sorted: &[Point3D],
        init_seeds: &mut Vec<Point3D>,
        th_seeds: f64,
    ) {
        init_seeds.clear();

        let mut sum = 0.0;
        let mut cnt = 0;
        let mut init_idx = 0;

        if zone_index == 0 {
            for i in 0..p_sorted.len() {
                if p_sorted[i].z
                    < (params.adaptive_seed_selection_margin * params.sensor_height) as f32
                {
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

        let lpr_height = if cnt != 0 { sum / (cnt as f32) } else { 0.0 };

        for i in 0..p_sorted.len() {
            if p_sorted[i].z < lpr_height + th_seeds as f32 {
                init_seeds.push(p_sorted[i].clone());
            }
        }
    }

    pub fn estimate_ground<
        R: Dim,
        C: Dim,
        S: std::fmt::Debug + RawStorage<f32, R, C> + nalgebra::RawStorageMut<f32, R, C>,
    >(
        &mut self,
        cloud_in: &mut nalgebra::Matrix<f32, R, C, S>,
    ) {
        self.cloud_ground.clear(); // ok
        self.cloud_nonground.clear(); // ok

        if self.params.verbose {
            // TODO: tracing
        }

        if self.params.enable_RNR {
            self.reflected_noise_removal(cloud_in);
        }

        self.flush_patches(); // ok

        self.pc2czm(cloud_in); // ok

        let mut concentric_idx = 0;

        self.centers.clear();
        self.normals.clear();

        let mut candidates = Vec::<RevertCandidate>::new();
        let mut ringwise_flatness = Vec::<f64>::new();

        let num_zones = self.params.num_zones as usize;

        // println!("num_zones = {}", num_zones);
        // println!(
        //     "num_rings_each_zone = {:?}",
        //     self.params.num_rings_each_zone
        // );
        // println!(
        //     "num_sectors_each_zone = {:?}",
        //     self.params.num_sectors_each_zone
        // );
        for zone_idx in 0..num_zones {
            for ring_idx in 0..self.params.num_rings_each_zone[zone_idx] {
                for sector_idx in 0..self.params.num_sectors_each_zone[zone_idx] {
                    let zone = &self.concentric_zone_model[zone_idx];

                    if zone[ring_idx][sector_idx].len() < self.params.num_min_pts {
                        self.cloud_nonground.add_cloud(&zone[ring_idx][sector_idx]);
                        continue;
                    }

                    // region-wise sorting (faster than global sorting method)

                    self.concentric_zone_model[zone_idx][ring_idx][sector_idx]
                        .sort_by(|x, y| x.compare_z(y));

                    // PCA
                    self.extract_piecewiseground(zone_idx, ring_idx, sector_idx);
                    self.centers.push(Point3D::new(
                        self.pc_mean[0],
                        self.pc_mean[1],
                        self.pc_mean[2],
                    ));
                    self.normals
                        .push(Point3D::new(self.normal[0], self.normal[1], self.normal[2]));

                    // GLE
                    let ground_uprightness = self.normal[2] as f64;
                    let ground_elevation = self.pc_mean[2] as f64;
                    let ground_flatness = self.singular_values.min() as f64;
                    let line_variable = if self.singular_values[1] != 0.0 {
                        (self.singular_values[0] / self.singular_values[1]) as f64
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
                    // println!(
                    //     "zone: {} ring: {} sector: {} is_upright: {}: heading: {} num_rings_of_interest: {} concentric_idx: {} elevation_thr: {:?} elevation_thr[{}]: {}",
                    //     zone_idx,
                    //     ring_idx,
                    //     sector_idx,
                    //     is_upright,
                    //     heading,
                    //     self.params.num_rings_of_interest,
                    //     concentric_idx,
                    //     self.params.elevation_thr,
                    //     concentric_idx,
                    //     self.params.elevation_thr.get(concentric_idx).copied().unwrap_or(f64::MIN),
                    // );
                    let is_near_zone = concentric_idx < self.params.num_rings_of_interest as usize;
                    let is_heading_outside = heading < 0.0;
                    let mut is_not_elevated = false;
                    let mut is_flat = false;
                    if concentric_idx < self.params.num_rings_of_interest as usize {
                        is_not_elevated =
                            ground_elevation < self.params.elevation_thr[concentric_idx];

                        is_flat = ground_flatness < self.params.flatness_thr[concentric_idx];
                    }

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
                        self.cloud_nonground.add_cloud(&self.regionwise_ground);
                    } else if !is_near_zone {
                        self.cloud_ground.add_cloud(&self.regionwise_ground);
                    } else if !is_heading_outside {
                        self.cloud_nonground.add_cloud(&self.regionwise_ground);
                    } else if is_not_elevated || is_flat {
                        self.cloud_ground.add_cloud(&self.regionwise_ground);
                    } else {
                        candidates.push(RevertCandidate::new(
                            concentric_idx as i32,
                            sector_idx as i32,
                            ground_flatness,
                            line_variable,
                            self.pc_mean.clone(),
                            self.regionwise_ground.clone(),
                        ));
                    }

                    self.cloud_nonground.add_cloud(&self.regionwise_nonground);
                }

                // Revert
                if !candidates.is_empty() {
                    if self.params.enable_TGR {
                        self.temporal_ground_revert(&ringwise_flatness, &candidates, concentric_idx)
                    } else {
                        for candidate in &candidates {
                            self.cloud_nonground.add_cloud(&candidate.regionwise_ground);
                        }
                    }
                    candidates.clear();
                    ringwise_flatness.clear();
                }

                concentric_idx += 1;
            }
        }
        self.update_elevation_thr();
        self.update_flatness_thr();
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
            if self.update_flatness[i].len() <= 1 {
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
                let a = candidate.ground_flatness - mu_flatness;
                let b = mu_flatness / 10.0;
                1.0 / (1.0 + (a / b).exp())
            };

            let prob_line = if candidate.line_variable > 8.0 {
                0.0
            } else {
                1.0
            };

            if concentric_idx < self.params.num_rings_of_interest as usize {
                let revert = prob_line * prob_flatness > 0.5;
                if revert {
                    self.cloud_ground.add_cloud(&candidate.regionwise_ground);
                } else {
                    self.cloud_nonground.add_cloud(&candidate.regionwise_ground);
                }
            }
        }
    }

    fn calc_mean_stdev(&self, v: &[f64]) -> (f64, f64) {
        if v.len() <= 1 {
            return (0.0, 0.0);
        }

        let mean = v.iter().sum::<f64>() / v.len() as f64;

        let stdev = v.iter().map(|&x| (x - mean) * (x - mean)).sum::<f64>() / (v.len() - 1) as f64;

        let stdev = stdev.sqrt();

        (mean, stdev)
    }

    fn reflected_noise_removal<
        R: Dim,
        C: Dim,
        S: std::fmt::Debug + RawStorage<f32, R, C> + nalgebra::RawStorageMut<f32, R, C>,
    >(
        &mut self,
        cloud_in: &mut nalgebra::Matrix<f32, R, C, S>,
    ) {
        if cloud_in.row(0).len() < 4 {
            return;
        }

        let mut cnt = 0;
        let mut line_numbers = vec![];
        for (i, mut row) in cloud_in.row_iter_mut().enumerate() {
            // double r = sqrt(
            //     cloud_in.row(i)(0) * cloud_in.row(i)(0) + cloud_in.row(i)(1) * cloud_in.row(i)(1));
            let a = row[0] as f64;
            let b = row[1] as f64;
            let r = (a * a + b * b).sqrt() as f64;
            let z = row[2] as f64;
            let ver_angle_in_deg = z.atan2(r).to_degrees() as f64;
            // println!("ver_angle_in_deg: {}", ver_angle_in_deg);
            if ver_angle_in_deg < self.params.RNR_ver_angle_thr
                && z < -self.params.sensor_height - 0.8
                && row[3] < self.params.RNR_intensity_thr as f32
            {
                //        cloud_nonground_.push_back(PointXYZ(cloud_in.row(i)(0), cloud_in.row(i)(1), cloud_in.row(i)(2)));
                // cloud_in.row(i)(2) = std::numeric_limits<float>::min();
                // cnt++;
                self.cloud_ground.push(Point3D::new(row[0], row[1], row[2]));
                row[2] = f32::MIN;
                println!(
                    "i = {} r = {} z = {} ver_angle_in_deg = {}",
                    i, r, z, ver_angle_in_deg
                );
                line_numbers.push(i);
                cnt += 1;
            }
        }
        // for num in line_numbers {
        //     println!("{:?}", debug_matrix("name", &cloud_in.row(num)));
        // }
        // println!("f64::MIN::cnt = {}", cnt);
        // std::process::exit(1);
        if self.params.verbose {
            // TODO: tracing
        }
    }

    fn pc2czm<R: Dim, C: Dim, S: std::fmt::Debug + RawStorage<f32, R, C>>(
        &mut self,
        src: &nalgebra::Matrix<f32, R, C, S>,
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

        for (_i, row) in src.row_iter().enumerate() {
            let x = row[0] as f64;
            let y = row[1] as f64;
            let z = row[2] as f64;

            let r = xy2radius(x, y);
            if (r <= max_range) && (r > min_range) {
                let theta = xy2theta(x, y);

                let index = if r < min_ranges[1] {
                    0
                } else if r < min_ranges[2] {
                    1
                } else if r < min_ranges[3] {
                    2
                } else {
                    3
                };

                let a = (r - min_ranges[index]) / self.ring_sizes[index];
                let ring_idx = (a as usize).min(num_ring[index] - 1);
                let b = theta / self.sector_sizes[index];
                let sector_idx = (b as usize).min(num_sector[index] - 1);
                // println!(
                //     "{}",
                //     serde_json::to_string(&serde_json::json!({
                //         "i": _i,
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
                    .push(Point3D::new(x as f32, y as f32, z as f32));
            } else {
                // println!("push to cloud_nonground. i = {}", _i);
                self.cloud_nonground
                    .push(Point3D::new(x as f32, y as f32, z as f32));
            }
        }
        // println!(
        //     "self.cloud_nonground.len() = {}",
        //     self.cloud_nonground.len()
        // );
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
                // let ground_pc = self.ground_pc.clone();
                self.estimate_plane(EstimatePlaneTarget::GroundPC);

                if zone_index == 0 && self.normal[2] < self.params.uprightness_thr as f32 {
                    let src_tmp = src_wo_verticals.clone();
                    src_wo_verticals.clear();

                    for point in src_tmp.iter() {
                        let distance =
                            crate::utils::calc_point_to_plane_d(&point, &self.normal, self.d);
                        if distance.abs() < self.params.th_dist_v {
                            self.regionwise_nonground.push(*point);
                        } else {
                            src_wo_verticals.push(*point);
                        }
                    }

                    // let src_tmp = &src_wo_verticals;
                    // let iter = src_tmp
                    //     .iter()
                    //     .map(|point| {
                    //         (
                    //             point,
                    //             crate::utils::calc_point_to_plane_d(&point, &self.normal, self.d),
                    //         )
                    //     })
                    //     .into_split_filter(|(_, distance)| distance.abs() < self.params.th_dist_v);

                    // let mut new_verticals = Points::from(vec![]);
                    // for item in iter {
                    //     match item {
                    //         (Some((point, _)), None) => {
                    //             self.regionwise_nonground.push(*point);
                    //         }
                    //         (None, Some((point, _))) => {
                    //             new_verticals.push(*point);
                    //         }
                    //         _ => unreachable!(),
                    //     }
                    // }
                    // src_wo_verticals = new_verticals;
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

        self.estimate_plane(EstimatePlaneTarget::GroundPC);

        for i in 0..self.params.num_iter {
            self.ground_pc.clear();

            for point in src_wo_verticals.iter() {
                let distance = crate::utils::calc_point_to_plane_d(point, &self.normal, self.d);

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
                self.estimate_plane(EstimatePlaneTarget::GroundPC);
            } else {
                self.estimate_plane(EstimatePlaneTarget::RegionwiseGround);
            }
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

pub struct RevertCandidate {
    concentric_idx: i32,
    sector_idx: i32,
    ground_flatness: f64,
    line_variable: f64,
    pc_mean: OVector<f32, nalgebra::Const<3>>,
    regionwise_ground: Points,
}

impl RevertCandidate {
    pub fn new(
        concentric_idx: i32,
        sector_idx: i32,
        ground_flatness: f64,
        line_variable: f64,
        pc_mean: OVector<f32, nalgebra::Const<3>>,
        regionwise_ground: Points,
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

use nalgebra::{Dim, MatrixXx3, OVector, RawStorage, Vector3};

use crate::{
    ground::Ground,
    utils::{xy2radius, xy2theta},
    Params, Point3D, Points,
};

#[derive(Debug)]
pub struct PatchWork {
    pub(crate) params: Params,
}

pub enum EstimatePlaneTarget {
    GroundPC,
    RegionwiseGround,
    Other(Points),
}

impl PatchWork {
    pub fn new(params: Params) -> Self {
        PatchWork { params }
    }
}

impl PatchWork {
    fn flush_patches(&self, ground: &mut Ground) {
        let czm = &mut ground.concentric_zone_model;
        for k in 0..self.params.num_zones {
            for i in 0..self.params.num_rings_each_zone[k] {
                for j in 0..self.params.num_sectors_each_zone[k] {
                    czm[k][i][j].clear();
                }
            }
        }
    }

    pub fn estimate_plane(&self, g: &mut Ground, target: EstimatePlaneTarget) {
        let ground = match target {
            EstimatePlaneTarget::GroundPC => &g.ground_pc,
            EstimatePlaneTarget::RegionwiseGround => &g.regionwise_ground,
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

        g.pc_mean[0] = mean[0];
        g.pc_mean[1] = mean[1];
        g.pc_mean[2] = mean[2];

        let centered = eigen_ground
            .row_iter()
            .map(|row| row - &mean)
            .collect::<Vec<_>>();

        let centered = MatrixXx3::from_rows(centered.as_slice());

        let cov = (centered.adjoint() * &centered) / (centered.nrows() as f32 - 1.0);

        // https://github.com/dimforge/nalgebra/issues/1072#issuecomment-1029808465
        // let mut svd = cov.try_svd(true, true, 4.0 * f64::EPSILON, 0).unwrap();
        let mut svd = nalgebra::SVD::new(cov, true, false);
        g.singular_values = svd.singular_values;

        let u = svd.u.take().unwrap();

        // v_t 就是转置过的矩阵，就是公式中的 A = U∑V^T 中的 V^T
        // 其中 ∑ 是一个对角矩阵，对角线上的元素就是 svd.singular_values
        // let v_t = svd.v_t.take().unwrap();

        let normal = u.column(2);
        g.normal = Vector3::new(normal[0], normal[1], normal[2]);

        if g.normal[2] < 0.0 {
            g.normal = -g.normal;
        }

        let seeds_mean = g.pc_mean;

        let d = g.normal.transpose() * seeds_mean;

        g.d = -d[0] as f64;
    }

    fn extract_initial_seeds(
        &self,
        zone_index: usize,
        p_sorted: &[Point3D],
        init_seeds: &mut Vec<Point3D>,
        sensor_height: f64,
    ) {
        init_seeds.clear();

        let mut sum = 0.0;
        let mut cnt = 0;
        let mut init_idx = 0;

        if zone_index == 0 {
            let threshold = self.params.adaptive_seed_selection_margin * sensor_height;
            for i in 0..p_sorted.len() {
                let z = p_sorted[i].z as f64;
                if z < threshold {
                    init_idx += 1;
                } else {
                    break;
                }
            }
        }

        // Calculate the mean height value.
        for i in init_idx..p_sorted.len() {
            if cnt >= self.params.num_lpr {
                break;
            }
            sum += p_sorted[i].z as f64;
            cnt += 1;
        }

        let lpr_height = if cnt != 0 { sum / (cnt as f64) } else { 0.0 };

        for i in 0..p_sorted.len() {
            let z = p_sorted[i].z as f64;
            if z < lpr_height + self.params.th_seeds {
                init_seeds.push(p_sorted[i].clone());
            }
        }
    }

    fn extract_initial_seeds_with_th_seed(
        &self,
        zone_index: usize,
        p_sorted: &[Point3D],
        init_seeds: &mut Vec<Point3D>,
        th_seeds: f64,
        sensor_height: f64,
    ) {
        init_seeds.clear();

        let mut sum = 0.0;
        let mut cnt = 0;
        let mut init_idx = 0;

        if zone_index == 0 {
            let threshold = self.params.adaptive_seed_selection_margin * sensor_height;
            for i in 0..p_sorted.len() {
                let z = p_sorted[i].z as f64;
                if z < threshold {
                    init_idx += 1;
                } else {
                    break;
                }
            }
        }

        // Calculate the mean height value.
        for i in init_idx..p_sorted.len() {
            if cnt >= self.params.num_lpr {
                break;
            }
            sum += p_sorted[i].z as f64;
            cnt += 1;
        }

        let lpr_height = if cnt != 0 { sum / (cnt as f64) } else { 0.0 };

        for i in 0..p_sorted.len() {
            let z = p_sorted[i].z as f64;
            if z < lpr_height + th_seeds {
                init_seeds.push(p_sorted[i].clone());
            }
        }
    }

    pub fn estimate_ground<
        R: Dim,
        C: Dim,
        S: std::fmt::Debug + RawStorage<f32, R, C> + nalgebra::RawStorageMut<f32, R, C>,
    >(
        &self,
        cloud_in: &mut nalgebra::Matrix<f32, R, C, S>,
    ) -> Ground {
        let mut ground = Ground::new(&self.params);

        if self.params.verbose {
            // TODO: tracing
        }

        if self.params.enable_RNR {
            self.reflected_noise_removal(&mut ground, cloud_in);
        }

        self.flush_patches(&mut ground);

        self.pc2czm(&mut ground, cloud_in);

        let mut concentric_idx = 0;

        ground.centers.clear();
        ground.normals.clear();

        let mut candidates = Vec::<RevertCandidate>::new();
        let mut ringwise_flatness = Vec::<f64>::new();

        let num_zones = self.params.num_zones as usize;

        for zone_idx in 0..num_zones {
            for ring_idx in 0..self.params.num_rings_each_zone[zone_idx] {
                for sector_idx in 0..self.params.num_sectors_each_zone[zone_idx] {
                    let zone = &ground.concentric_zone_model[zone_idx];

                    if zone[ring_idx][sector_idx].len() < self.params.num_min_pts {
                        ground
                            .cloud_nonground
                            .add_cloud(&zone[ring_idx][sector_idx]);
                        continue;
                    }

                    // region-wise sorting (faster than global sorting method)

                    ground.concentric_zone_model[zone_idx][ring_idx][sector_idx]
                        .sort_by(|x, y| x.compare_z(y));

                    // PCA
                    self.extract_piecewiseground(&mut ground, zone_idx, ring_idx, sector_idx);
                    ground.centers.push(Point3D::new(
                        ground.pc_mean[0],
                        ground.pc_mean[1],
                        ground.pc_mean[2],
                    ));
                    ground.normals.push(Point3D::new(
                        ground.normal[0],
                        ground.normal[1],
                        ground.normal[2],
                    ));

                    // GLE
                    let ground_uprightness = ground.normal[2] as f64;
                    let ground_elevation = ground.pc_mean[2] as f64;
                    let ground_flatness = ground.singular_values.min() as f64;
                    let line_variable = if ground.singular_values[1] != 0.0 {
                        (ground.singular_values[0] / ground.singular_values[1]) as f64
                    } else {
                        // https://en.cppreference.com/w/cpp/types/climits
                        std::f64::MAX
                    };
                    let mut heading = 0.0;
                    for i in 0..3 {
                        heading = heading + (ground.pc_mean[i] * ground.normal[i]);
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
                    let is_near_zone = concentric_idx < self.params.num_rings_of_interest as usize;
                    let is_heading_outside = heading < 0.0;
                    let mut is_not_elevated = false;
                    let mut is_flat = false;
                    if concentric_idx < self.params.num_rings_of_interest as usize {
                        is_not_elevated = ground_elevation < ground.elevation_thr[concentric_idx];

                        is_flat = ground_flatness < ground.flatness_thr[concentric_idx];
                    }

                    // Store the elevation & flatness variables
                    // for A-GLE (Adaptive Ground Likelihood Estimation)
                    // and TGR (Temporal Ground Revert). More information in the paper Patchwork++.

                    if is_upright && is_not_elevated && is_near_zone {
                        ground.update_elevation[concentric_idx].push(ground_elevation);
                        ground.update_flatness[concentric_idx].push(ground_flatness);

                        ringwise_flatness.push(ground_flatness);
                    }

                    // Ground estimation based on conditions
                    if !is_upright {
                        ground.cloud_nonground.add_cloud(&ground.regionwise_ground);
                    } else if !is_near_zone {
                        ground.cloud_ground.add_cloud(&ground.regionwise_ground);
                    } else if !is_heading_outside {
                        ground.cloud_nonground.add_cloud(&ground.regionwise_ground);
                    } else if is_not_elevated || is_flat {
                        ground.cloud_ground.add_cloud(&ground.regionwise_ground);
                    } else {
                        candidates.push(RevertCandidate::new(
                            concentric_idx as i32,
                            sector_idx as i32,
                            ground_flatness,
                            line_variable,
                            ground.pc_mean.clone(),
                            ground.regionwise_ground.clone(),
                        ));
                    }

                    ground
                        .cloud_nonground
                        .add_cloud(&ground.regionwise_nonground);
                }

                // Revert
                if !candidates.is_empty() {
                    if self.params.enable_TGR {
                        self.temporal_ground_revert(
                            &mut ground,
                            &ringwise_flatness,
                            &candidates,
                            concentric_idx,
                        )
                    } else {
                        println!(
                            "zone = {zone_idx} ring = {ring_idx} candidates.len = {}",
                            candidates.len()
                        );
                        for candidate in &candidates {
                            ground
                                .cloud_nonground
                                .add_cloud(&candidate.regionwise_ground);
                        }
                    }
                    candidates.clear();
                    ringwise_flatness.clear();
                }

                concentric_idx += 1;
            }
        }
        self.update_elevation_thr(&mut ground);
        self.update_flatness_thr(&mut ground);

        ground
    }

    fn update_elevation_thr(&self, ground: &mut Ground) {
        for i in 0..self.params.num_rings_of_interest as usize {
            if ground.update_elevation[i].is_empty() {
                continue;
            }
            let (update_mean, update_stdev) = self.calc_mean_stdev(&ground.update_elevation[i]);
            if i == 0 {
                ground.elevation_thr[i] = update_mean + 3.0 * update_stdev;
                ground.sensor_height = -update_mean;
            } else {
                ground.elevation_thr[i] = update_mean + 2.0 * update_stdev;
            }

            let exceed_num =
                ground.update_elevation[i].len() as i32 - self.params.max_elevation_storage;
            if exceed_num > 0 {
                ground.update_elevation[i].drain(0..exceed_num as usize);
            }
        }
    }

    fn update_flatness_thr(&self, ground: &mut Ground) {
        for i in 0..self.params.num_rings_of_interest as usize {
            if ground.update_flatness[i].is_empty() {
                break;
            }
            if ground.update_flatness[i].len() <= 1 {
                break;
            }

            let (update_mean, update_stdev) = self.calc_mean_stdev(&ground.update_flatness[i]);
            ground.flatness_thr[i] = update_mean + update_stdev;

            let exceed_num =
                ground.update_flatness[i].len() as i32 - self.params.max_flatness_storage;
            if exceed_num > 0 {
                ground.update_flatness[i].drain(0..exceed_num as usize);
            }
        }
    }

    fn temporal_ground_revert(
        &self,
        ground: &mut Ground,
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
                    ground.cloud_ground.add_cloud(&candidate.regionwise_ground);
                } else {
                    ground
                        .cloud_nonground
                        .add_cloud(&candidate.regionwise_ground);
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
        &self,
        ground: &mut Ground,
        cloud_in: &mut nalgebra::Matrix<f32, R, C, S>,
    ) {
        if cloud_in.row(0).len() < 4 {
            return;
        }

        let mut line_numbers = vec![];
        for (i, mut row) in cloud_in.row_iter_mut().enumerate() {
            let a = row[0] as f64;
            let b = row[1] as f64;
            let r = (a * a + b * b).sqrt() as f64;
            let z = row[2] as f64;
            let ver_angle_in_deg = z.atan2(r).to_degrees() as f64;

            if ver_angle_in_deg < self.params.RNR_ver_angle_thr
                && z < -ground.sensor_height - 0.8
                && row[3] < self.params.RNR_intensity_thr as f32
            {
                ground
                    .cloud_nonground
                    .push(Point3D::new(row[0], row[1], row[2]));
                row[2] = f32::NAN;
                line_numbers.push(i);
            }
        }

        if self.params.verbose {
            // TODO: tracing
        }
    }

    fn pc2czm<R: Dim, C: Dim, S: std::fmt::Debug + RawStorage<f32, R, C>>(
        &self,
        ground: &mut Ground,
        src: &nalgebra::Matrix<f32, R, C, S>,
    ) {
        let max_range = self.params.max_range;
        let min_range = self.params.min_range;

        let min_ranges = [
            ground.min_ranges[0],
            ground.min_ranges[1],
            ground.min_ranges[2],
            ground.min_ranges[3],
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

            if z.is_nan() {
                continue;
            }

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

                let a = (r - min_ranges[index]) / ground.ring_sizes[index];
                let ring_idx = (a as usize).min(num_ring[index] - 1);
                let b = theta / ground.sector_sizes[index];
                let sector_idx = (b as usize).min(num_sector[index] - 1);
                ground.concentric_zone_model[index][ring_idx][sector_idx]
                    .push(Point3D::new(x as f32, y as f32, z as f32));
            } else {
                ground
                    .cloud_nonground
                    .push(Point3D::new(x as f32, y as f32, z as f32));
            }
        }
    }

    fn extract_piecewiseground(
        &self,
        ground: &mut Ground,
        zone_index: usize,
        ring_index: usize,
        sector_index: usize,
    ) {
        if !ground.ground_pc.is_empty() {
            ground.ground_pc.clear();
        }

        let dst = &mut ground.regionwise_ground;
        if !dst.is_empty() {
            dst.clear();
        }

        let non_ground_dst = &mut ground.regionwise_nonground;
        if !non_ground_dst.is_empty() {
            non_ground_dst.clear();
        }

        let mut src_wo_verticals =
            ground.concentric_zone_model[zone_index][ring_index][sector_index].clone();

        if self.params.enable_RVPF {
            for _i in 0..self.params.num_iter {
                self.extract_initial_seeds_with_th_seed(
                    zone_index,
                    &src_wo_verticals,
                    &mut ground.ground_pc,
                    self.params.th_seeds_v,
                    ground.sensor_height,
                );
                self.estimate_plane(ground, EstimatePlaneTarget::GroundPC);

                if zone_index == 0 && ground.normal[2] < self.params.uprightness_thr as f32 {
                    let src_tmp = src_wo_verticals.clone();
                    src_wo_verticals.clear();

                    for point in src_tmp.iter() {
                        let distance =
                            crate::utils::calc_point_to_plane_d(&point, &ground.normal, ground.d);
                        if distance.abs() < self.params.th_dist_v {
                            ground.regionwise_nonground.push(*point);
                        } else {
                            src_wo_verticals.push(*point);
                        }
                    }
                } else {
                    break;
                }
            }
        }

        self.extract_initial_seeds(
            zone_index,
            &src_wo_verticals,
            &mut ground.ground_pc,
            ground.sensor_height,
        );

        self.estimate_plane(ground, EstimatePlaneTarget::GroundPC);

        for i in 0..self.params.num_iter {
            ground.ground_pc.clear();

            for point in src_wo_verticals.iter() {
                let distance = crate::utils::calc_point_to_plane_d(point, &ground.normal, ground.d);

                if i < self.params.num_iter - 1 {
                    if distance < self.params.th_dist {
                        ground.ground_pc.push(*point);
                    }
                } else {
                    if distance < self.params.th_dist {
                        ground.regionwise_ground.push(*point);
                    } else {
                        ground.regionwise_nonground.push(*point);
                    }
                }
            }

            if i < self.params.num_iter - 1 {
                self.estimate_plane(ground, EstimatePlaneTarget::GroundPC);
            } else {
                self.estimate_plane(ground, EstimatePlaneTarget::RegionwiseGround);
            }
        }
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

pub fn nx4f64_from_raw_buffer(
    mut buffer: Vec<u8>,
) -> nalgebra::Matrix<
    f32,
    nalgebra::Dyn,
    nalgebra::Const<4>,
    nalgebra::VecStorage<f32, nalgebra::Dyn, nalgebra::Const<4>>,
> {
    assert!(buffer.len() % 4 == 0);

    let buffer_length = buffer.len();
    let points = {
        let len = buffer_length / 4;
        unsafe {
            let v = Vec::from_raw_parts(buffer.as_mut_ptr() as *mut f32, len, len);
            std::mem::forget(buffer);
            v
        }
    };
    let num_points = points.len() / 4;
    let mut mat: nalgebra::Matrix<
        f32,
        nalgebra::Dyn,
        nalgebra::Const<4>,
        nalgebra::VecStorage<f32, nalgebra::Dyn, nalgebra::Const<4>>,
    > = nalgebra::MatrixXx4::<f32>::zeros(num_points);
    for i in 0..num_points {
        mat[(i, 0)] = points[i * 4] as f32;
        mat[(i, 1)] = points[i * 4 + 1] as f32;
        mat[(i, 2)] = points[i * 4 + 2] as f32;
        mat[(i, 3)] = points[i * 4 + 3] as f32;
    }
    mat
}

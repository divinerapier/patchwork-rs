pub fn nx4f32_from_raw_buffer(
    buffer: Vec<u8>,
) -> nalgebra::Matrix<
    f32,
    nalgebra::Dyn,
    nalgebra::Const<4>,
    nalgebra::VecStorage<f32, nalgebra::Dyn, nalgebra::Const<4>>,
> {
    matrix_from_raw_buffer::<4>(buffer)
}

pub fn nx3f32_from_raw_buffer(
    buffer: Vec<u8>,
) -> nalgebra::Matrix<
    f32,
    nalgebra::Dyn,
    nalgebra::Const<3>,
    nalgebra::VecStorage<f32, nalgebra::Dyn, nalgebra::Const<3>>,
> {
    matrix_from_raw_buffer::<3>(buffer)
}

/// This function takes a buffer of bytes and returns a matrix of floats.
/// The buffer is assumed to be a sequence of floats in row-major order.
pub fn matrix_from_raw_buffer<const N: usize>(
    mut buffer: Vec<u8>,
) -> nalgebra::Matrix<
    f32,
    nalgebra::Dyn,
    nalgebra::Const<N>,
    nalgebra::VecStorage<f32, nalgebra::Dyn, nalgebra::Const<N>>,
> {
    assert!(buffer.len() % N == 0);

    let buffer_length = buffer.len();
    let points = {
        let len = buffer_length / 4;
        unsafe {
            let v = Vec::from_raw_parts(buffer.as_mut_ptr() as *mut f32, len, len);
            std::mem::forget(buffer);
            v
        }
    };
    let num_points = points.len() / N;
    let mut mat: nalgebra::Matrix<
        f32,
        nalgebra::Dyn,
        nalgebra::Const<N>,
        nalgebra::VecStorage<f32, nalgebra::Dyn, nalgebra::Const<N>>,
    > = nalgebra::Matrix::<f32, nalgebra::Dyn, nalgebra::Const<N>, _>::zeros(num_points);
    for i in 0..num_points {
        for col in 0..N {
            mat[(i, col)] = points[i * N + col] as f32;
        }
    }
    mat
}

/// This function is the same as `matrix_from_raw_buffer` except that it drops the last column.
/// This is useful for reading in a matrix of points from a file where the last column is
/// a label or some other data that we don't want to include in the matrix.
pub fn matrix_from_raw_buffer_drop_last<const N: usize>(
    mut buffer: Vec<u8>,
) -> nalgebra::Matrix<
    f32,
    nalgebra::Dyn,
    nalgebra::Const<N>,
    nalgebra::VecStorage<f32, nalgebra::Dyn, nalgebra::Const<N>>,
> {
    assert!(buffer.len() % (N + 1) == 0);

    let buffer_length = buffer.len();
    let points = {
        let len = buffer_length / 4; // 4 bytes per float
        unsafe {
            let v = Vec::from_raw_parts(buffer.as_mut_ptr() as *mut f32, len, len);
            std::mem::forget(buffer);
            v
        }
    };
    let num_points = points.len() / (N + 1); // N floats per point
    let mut mat: nalgebra::Matrix<
        f32,
        nalgebra::Dyn,
        nalgebra::Const<N>,
        nalgebra::VecStorage<f32, nalgebra::Dyn, nalgebra::Const<N>>,
    > = nalgebra::Matrix::<f32, nalgebra::Dyn, nalgebra::Const<N>, _>::zeros(num_points);
    for i in 0..num_points {
        for col in 0..(N + 1) {
            if col != N {
                mat[(i, col)] = points[i * (N + 1) + col] as f32;
            }
        }
    }
    mat
}

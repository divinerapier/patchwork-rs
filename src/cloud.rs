pub trait IntoCloud {
    fn into_cloud(self) -> nalgebra::MatrixXx3<f32>;
}

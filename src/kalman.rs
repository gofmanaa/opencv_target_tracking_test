use opencv::core::CV_32F;
use opencv::{
    Result,
    core::{Mat, Point2f},
    prelude::*,
    video::KalmanFilter,
};

pub struct KalmanTracker {
    kf: KalmanFilter,
    state: Mat, // state vector [x, y, dx, dy]
    meas: Mat,  // measurement vector [x, y]
}

impl KalmanTracker {
    pub fn new() -> Result<Self> {
        let mut kf = KalmanFilter::new(4, 2, 0, CV_32F)?; // 4 dynamic params, 2 measured params, 0 control params

        // Transition matrix A
        let transition_matrix = Mat::from_slice_2d(&[
            [1f32, 0f32, 1f32, 0f32],
            [0f32, 1f32, 0f32, 1f32],
            [0f32, 0f32, 1f32, 0f32],
            [0f32, 0f32, 0f32, 1f32],
        ])?;
        kf.set_transition_matrix(transition_matrix);

        // Initialize state and measurement matrices
        let state = Mat::zeros(4, 1, opencv::core::CV_32F)?.to_mat()?;
        let meas = Mat::zeros(2, 1, opencv::core::CV_32F)?.to_mat()?;

        // Set measurement matrix H
        let measurement_matrix =
            Mat::from_slice_2d(&[[1f32, 0f32, 0f32, 0f32], [0f32, 1f32, 0f32, 0f32]])?;
        kf.set_measurement_matrix(measurement_matrix);

        // Process noise covariance Q
        let mut q = Mat::eye(4, 4, opencv::core::CV_32F)?.to_mat()?;
        *q.at_2d_mut::<f32>(0, 0)? = 1e-2;
        *q.at_2d_mut::<f32>(1, 1)? = 1e-2;
        *q.at_2d_mut::<f32>(2, 2)? = 5.0;
        *q.at_2d_mut::<f32>(3, 3)? = 5.0;
        kf.set_process_noise_cov(q);

        // Measurement noise covariance R
        let mut r = Mat::eye(2, 2, opencv::core::CV_32F)?.to_mat()?;
        *r.at_2d_mut::<f32>(0, 0)? = 1e-1;
        *r.at_2d_mut::<f32>(1, 1)? = 1e-1;
        kf.set_measurement_noise_cov(r);

        Ok(KalmanTracker { kf, state, meas })
    }

    pub fn init(&mut self, point: Point2f) -> Result<()> {
        // Initialize state with position and zero velocity
        *self.state.at_2d_mut::<f32>(0, 0)? = point.x;
        *self.state.at_2d_mut::<f32>(1, 0)? = point.y;
        *self.state.at_2d_mut::<f32>(2, 0)? = 0.0;
        *self.state.at_2d_mut::<f32>(3, 0)? = 0.0;
        let new_state = self.state.clone();
        self.kf.set_state_post(new_state);
        Ok(())
    }

    pub fn predict(&mut self) -> Result<Point2f> {
        self.state = self.kf.predict(&Mat::default())?;
        let x = *self.state.at_2d::<f32>(0, 0)?;
        let y = *self.state.at_2d::<f32>(1, 0)?;
        Ok(Point2f::new(x, y))
    }

    pub fn correct(&mut self, point: Point2f) -> Result<Point2f> {
        *self.meas.at_2d_mut::<f32>(0, 0)? = point.x;
        *self.meas.at_2d_mut::<f32>(1, 0)? = point.y;
        self.state = self.kf.correct(&self.meas)?;
        let x = *self.state.at_2d::<f32>(0, 0)?;
        let y = *self.state.at_2d::<f32>(1, 0)?;
        Ok(Point2f::new(x, y))
    }
}

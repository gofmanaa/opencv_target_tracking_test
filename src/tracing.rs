use crate::kalman::KalmanTracker;
use opencv::core::Point2f;
use opencv::{
    core::{Rect, Scalar},
    highgui, imgproc,
    prelude::*,
    tracking::{self, TrackerCSRT},
};
use std::sync::{Arc, Mutex};

#[derive(Clone)]
pub struct Tracing {
    roi: Arc<Mutex<Option<Rect>>>,
    tracker: Arc<Mutex<Option<opencv::core::Ptr<TrackerCSRT>>>>,
    kalman: Arc<Mutex<Option<KalmanTracker>>>,

    fail_count: i32,
    fail_threshold: i32,

    prev_corrected_position: Option<Point2f>,
}

impl Tracing {
    pub fn new() -> Self {
        Self {
            roi: Arc::new(Mutex::new(None)),
            tracker: Arc::new(Mutex::new(None)),
            kalman: Arc::new(Mutex::new(None)),
            fail_threshold: 50,
            fail_count: 0,
            prev_corrected_position: None,
        }
    }

    pub fn mouse_callback(&mut self) -> highgui::MouseCallback {
        let roi = Arc::clone(&self.roi);
        Some(Box::new(move |event, x, y, _flags| {
            if event == highgui::EVENT_LBUTTONDOWN {
                let mut roi_lock = roi.lock().unwrap();
                *roi_lock = Some(Rect::new(x - 30, y - 30, 60, 60));
            }
        }))
    }

    pub fn get_roi_pointer(&self) -> Arc<Mutex<Option<Rect>>> {
        Arc::clone(&self.roi)
    }

    pub fn set_roi_from_rect(&mut self, roi: Rect) {
        let mut roi_lock = self.roi.lock().unwrap();
        *roi_lock = Some(roi);
    }

    pub fn set_roi_from_point(&mut self, x: i32, y: i32) {
        let roi_rect = Rect::new(x - 30, y - 30, 60, 60);
        if let Ok(mut roi) = self.roi.lock() {
            *roi = Some(roi_rect);
        }
    }
    pub fn update(&mut self, frame: &mut Mat) -> opencv::Result<()> {
        // Check for new ROI and create tracker
        if let Some(roi) = self.roi.lock().unwrap().take() {
            let params = tracking::TrackerCSRT_Params::default()?;
            let mut new_tracker = TrackerCSRT::create(&params)?;
            new_tracker.init(frame, roi)?;
            *self.tracker.lock().unwrap() = Some(new_tracker);

            let mut kf = KalmanTracker::new()?;
            let center = Point2f::new(
                (roi.x + roi.width / 2) as f32,
                (roi.y + roi.height / 2) as f32,
            );
            kf.init(center)?;
            *self.kalman.lock().unwrap() = Some(kf);
        }

        // Update existing tracker
        if let Some(ref mut tracker) = *self.tracker.lock().unwrap() {
            let mut tracked = Rect::default();
            if tracker.update(frame, &mut tracked)? {
                self.fail_count = 0;
                let center = Point2f::new(
                    (tracked.x + tracked.width / 2) as f32,
                    (tracked.y + tracked.height / 2) as f32,
                );

                if let Some(kf) = &mut *self.kalman.lock().unwrap() {
                    let predicted = kf.predict()?;
                    let corrected = kf.correct(center)?;

                    let predicted_point =
                        opencv::core::Point::new(predicted.x as i32, predicted.y as i32);
                    // Draw predicted position (blue circle)
                    imgproc::circle(
                        frame,
                        predicted_point,
                        5,
                        Scalar::new(255.0, 0.0, 0.0, 0.0),
                        -1,
                        imgproc::LINE_8,
                        0,
                    )?;

                    let corrected_point =
                        opencv::core::Point::new(corrected.x as i32, corrected.y as i32);
                    // Draw corrected (tracked) position (green circle)
                    imgproc::circle(
                        frame,
                        opencv::core::Point::new(corrected.x as i32, corrected.y as i32),
                        5,
                        Scalar::new(0.0, 255.0, 0.0, 0.0),
                        -1,
                        imgproc::LINE_8,
                        0,
                    )?;

                    if let Some(prev) = self.prev_corrected_position {
                        let prev_point = opencv::core::Point::new(prev.x as i32, prev.y as i32);
                        imgproc::arrowed_line(
                            frame,
                            prev_point,
                            corrected_point,
                            Scalar::new(0.0, 255.0, 255.0, 0.0), // Cyan color for arrow
                            2,
                            imgproc::LINE_8,
                            0,
                            0.3, // tip length ratio
                        )?;
                    }
                    self.prev_corrected_position = Some(corrected)
                }

                imgproc::rectangle(
                    frame,
                    tracked,
                    Scalar::new(0.0, 255.0, 0.0, 0.0),
                    2,
                    imgproc::LINE_8,
                    0,
                )?;
            } else {
                println!("Tracing loos...");
                self.fail_count += 1;
                if self.fail_count < self.fail_threshold {
                    if let Some(kf) = &mut *self.kalman.lock().unwrap() {
                        let predicted = kf.predict()?;
                        let predicted_point =
                            opencv::core::Point::new(predicted.x as i32, predicted.y as i32);
                        // Draw predicted position (red circle)
                        imgproc::circle(
                            frame,
                            predicted_point,
                            5,
                            Scalar::new(0.0, 0.0, 255.0, 0.0),
                            -1,
                            imgproc::LINE_8,
                            0,
                        )?;

                        // Optionally, draw a rectangle around predicted position
                        let pred_rect =
                            Rect::new(predicted.x as i32 - 30, predicted.y as i32 - 30, 60, 60);
                        imgproc::rectangle(
                            frame,
                            pred_rect,
                            Scalar::new(0.0, 0.0, 0.0, 0.0),
                            2,
                            imgproc::LINE_8,
                            0,
                        )?;
                    }
                } else {
                    self.fail_count = 0;
                }
            }
        }
        Ok(())
    }
}

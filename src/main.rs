use opencv::{highgui, prelude::*, videoio};

mod kalman;
mod tracing;

use tracing::Tracing;

#[tokio::main]
async fn main() -> opencv::Result<()> {
    let mut cam = videoio::VideoCapture::new(0, videoio::CAP_ANY)?;
    if !videoio::VideoCapture::is_opened(&cam)? {
        panic!("Cannot open camera");
    }

    let mut tracing = Tracing::new();
    let window = "tracking";
    highgui::named_window(window, highgui::WINDOW_AUTOSIZE)?;

    // Set mouse callback to capture ROI
    highgui::set_mouse_callback(window, tracing.mouse_callback())?;

    let mut frame = Mat::default();

    loop {
        cam.read(&mut frame)?;
        if frame.empty() {
            continue;
        }

        tracing.update(&mut frame)?;

        highgui::imshow(window, &frame)?;
        if highgui::wait_key(30)? == 27 {
            break;
        }
    }

    Ok(())
}

use opencv::{
    highgui,
    prelude::*,
    videoio,
};

mod tracing;
mod kalman;

use tracing::Tracing;

fn main() -> opencv::Result<()> {
    // let mut cam = videoio::VideoCapture::new(0, videoio::CAP_ANY)?;
    // if !videoio::VideoCapture::is_opened(&cam)? {
    //     panic!("Cannot open camera");
    // }
    let mut cam = videoio::VideoCapture::from_file("/home/sasha/Videos/war/IMG_5740.MP4", videoio::CAP_ANY)?;
    if !videoio::VideoCapture::is_opened(&cam)? {
        panic!("Cannot open video file");
    }
    let mut tracing = Tracing::new();
    let window = "tracking";
    highgui::named_window(window, highgui::WINDOW_AUTOSIZE)?;

    // Set mouse callback to capture ROI
    highgui::set_mouse_callback(
        window,
        tracing.mouse_callback()
    )?;

    let mut frame = Mat::default();

    loop {
        cam.read(&mut frame)?;
        if frame.empty() {
            // continue;
            println!("End of video or cannot read frame");
            break; // exit loop gracefully
        }

        tracing.update(&mut frame)?;

        highgui::imshow(window, &frame)?;
        if highgui::wait_key(30)? == 27 {
            break;
        }
    }

    Ok(())
}

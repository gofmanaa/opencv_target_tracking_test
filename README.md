# 📹 Overview

This project performs real-time object tracking on video input using **OpenCV in Rust**. The tracker is enhanced with a **Kalman Filter** to handle temporary occlusions and predict future object positions, improving robustness and continuity.

---

## ⚙️ How It Works

1. 🖱️ **Click on the object** you want to track in the video window.
2. A **region of interest (ROI)** will be selected around the mouse position.
3. The **CSRT tracker** is initialized to follow that ROI across frames.
4. A **Kalman Filter** is started to:
  - Smooth noisy positions
  - Predict object motion during short-term occlusion
5. If tracking fails (e.g., object is hidden), the **Kalman Filter predicts** the next position.
6. The object’s motion direction and speed are shown using an **arrow**:
  - **Direction** = estimated motion vector
  - **Length** = current speed

This design allows robust object tracking even with partial occlusion or temporary loss of detection.

---

## 📁 Project Structure

- `main.rs` – Loads video input, GUI loop
- `tracing.rs` – Tracker logic and Kalman integration
- `kalman.rs` – Kalman Filter definition and processing

---

## 🛠 Tech Stack

- **Rust**
- **OpenCV** via [opencv-rust](https://github.com/twistedfall/opencv-rust)
- **Multithreading** with `Mutex` for shared state
- **Kalman Filter** for motion estimation

---

## 🚀 Building and Running

To build:

```bash
cargo build --release
```

To run the project:

```bash
cargo run --release
```


## Dependencies

This project uses the following main dependencies:

- `opencv`: For computer vision tasks.
- `tokio`: Asynchronous runtime.

## License

This project is licensed under the MIT License.

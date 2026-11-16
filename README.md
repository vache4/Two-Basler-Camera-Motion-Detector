# Dual Basler Motion Capture

A two-camera machine vision project for recording short motion events from two viewpoints. The original setup uses two Basler USB cameras mounted at right angles to estimate where a badminton shuttlecock lands in a test area. The repository includes a Python implementation and a C++17 port for Windows.

## How it works

```text
Camera A (motion view) ── frame difference ── trigger image ─┐
                                                            ├── paired PNGs + event CSV
Camera B (measurement view) ── latest frame ── detection ───┘
```

Both cameras stream continuously. Camera A compares successive grayscale frames and starts an event when enough pixels change. The event worker saves the exact Camera A trigger frame and the most recent Camera B frame available when the worker reads it. On that Camera B image, the detector finds a bright circular skirt and, if visible, a smaller dark base inside it. It measures each center independently. A cooldown limits repeated triggers from the same motion.

The pairing is done in software. Camera B is neither fired by Camera A nor synchronized to it. A recorded position estimates where the shuttlecock appears in the selected frame. To claim impact coordinates, verify timing and calibration against known landing positions.

## Start here

1. Install [Basler pylon](https://www.baslerweb.com/en/software/pylon/) and confirm that both cameras stream individually in pylon Viewer. Close the viewer before running the application.
2. Choose [Python installation](docs/installation.md#python) or [C++ build on Windows](docs/installation.md#c-on-windows).
3. Set the two serial numbers in the chosen source file and follow the [camera setup and calibration guide](docs/setup.md).
4. Run from the repository root. Generate a test event, then check both PNGs and the corresponding CSV row. Press **Q** or **Esc** in a preview window to quit.

| Path | Purpose |
| --- | --- |
| [python/](python/) | Python implementation using pypylon, OpenCV, NumPy and sounddevice |
| [cpp/](cpp/) | C++17 implementation using the pylon SDK and OpenCV |
| [CMakeLists.txt](CMakeLists.txt) | C++ build configuration |
| [requirements.txt](requirements.txt) | Python dependencies |
| [docs/](docs/) | Installation, calibration, operation and troubleshooting |

The Python version requests automatic gain once, uses a 1% motion area threshold and waits five seconds before rearming. The C++ version requests fixed gain, blurs frames before differencing, uses a 2% motion area threshold and waits one second. Their circle filters also differ. Use the same test scene to tune each independently.

## Adapting the project

Keep the trigger, buffered capture and event logging, then replace the Camera B detector and CSV fields for the measurement you need. For example:

| Application | Camera A trigger | Camera B measurement |
| --- | --- | --- |
| Conveyor inspection | A part enters the inspection zone | Check its position, orientation or surface |
| Lab drop test | A specimen passes a narrow region | Estimate where it strikes a marked target |
| Packaging station | A package arrives at the fixture | Read a label or verify its placement |

In Python, the application-specific entry point is `detect_and_annotate`; in C++ it is `detectShuttlecock`. If exact timing is required, use a shared hardware trigger or match camera timestamps, and measure the residual timing error.

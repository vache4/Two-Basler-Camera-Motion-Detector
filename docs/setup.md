# Camera setup, calibration and outputs

1. **Mount and identify.** Aim Camera A at a narrow zone where movement should start an event. Aim Camera B at the measurement area. The original badminton rig mounts the views perpendicular to one another and uses Camera B to see the shuttlecock from behind. Confirm the serial-to-role mapping using pylon Viewer.
2. **Frame and light the scene.** The code requests Mono8, 100 fps and 100 µs exposure, with centered sensor crops of 100 rows for A and 400 for B. Both streams rotate 90° counterclockwise before analysis and saving. Use steady light, a rigid mount and a background that contrasts with the shuttlecock. Check camera limits for the requested frame rate, exposure, height and offset.
3. **Set the coordinate scale.** Put a ruler or calibration target in the expected landing plane. Measure the pixel distance between two marks in the rotated Camera B image. Set `MM_PER_PX` to known distance in millimeters divided by measured distance in pixels. The initial 550/400 value is an example from the original rig. Recheck calibration after changing the camera position, lens, crop or binning.
4. **Tune and verify.** Trigger a few known landings and compare the CSV with the saved Camera B images and marked points. Adjust `MOTION_THRESHOLD` and `MOTION_MIN_AREA_FRAC` for false or missed triggers. Use the preview masks to tune brightness, circularity and radius limits if the skirt or base is missing. Adjust `REARM_DELAY_S` to fit the time between events.

The CSV uses the top left of the rotated Camera B image as its origin; x grows to the right and y grows downward. The current conversion multiplies image measurements by one scale. An oblique view, lens distortion or movement out of the calibration plane can bias the result. To express coordinates on a court or another physical plane, map known reference points to that plane, using lens correction and a planar homography as needed.

## Files written per run

Python writes to `captures3/` and C++ to `captures2/`, relative to the working directory. Each run has a timestamped image directory and a sibling `<run>_coords.csv`. Python creates the CSV on the first processed event; C++ creates it at startup.

| Output | Meaning |
| --- | --- |
| `a001.png` | Camera A frame that triggered event 1 |
| `b001.png` | Latest Camera B frame read by the event worker, if one was available |
| `event`, `timestamp` | Event number within the run and local logging time, not camera exposure time |
| `white_x_mm`, `white_y_mm`, `white_radius_mm` | Bright skirt circle |
| `black_x_mm`, `black_y_mm`, `black_radius_mm` | Dark base circle |
| `center_distance_mm` | Distance between detected centers, when both are present |

A processed event with no detection has blank measurement fields. If only the skirt was found, the base and distance fields are blank. A saved Camera B image can therefore exist even when no coordinates were detected. PNGs are saved without preview annotations.

## Troubleshooting

| Symptom | Check |
| --- | --- |
| Camera not found or cannot open | Verify serials and cables in pylon Viewer, then close the viewer and retry. |
| GenICam node or range error | Check feature availability, crop increments and exposure/frame-rate limits for the camera model. |
| Motion triggers too often | Stabilize lighting and the camera, then raise the motion threshold or required changed area. |
| PNG exists but coordinates are blank | Check the Camera B view and masks for contrast, apparent size, roundness and visibility of the dark base. |
| Position shifts at higher speeds | Inspect the saved frame pair. Buffer timing, perspective and exposure blur can all contribute to the error. |
| No preview | Use a desktop session and OpenCV with GUI support. |

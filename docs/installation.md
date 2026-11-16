# Installation and build

Run commands from the repository root. Use two supported Basler cameras with compatible GenICam features, a desktop session for OpenCV previews and a writable working directory. Verify both devices in pylon Viewer before launching either version.

## Python

Use a 64-bit Python with wheels available for your OS and architecture; see [Basler's pypylon installation matrix](https://github.com/basler/pypylon#installation). Use the GUI-enabled `opencv-python` package rather than its headless variant. If sounddevice cannot load PortAudio, see its [installation instructions](https://python-sounddevice.readthedocs.io/en/latest/installation.html).

Set `CAMERA_A_SERIAL` and `CAMERA_B_SERIAL` at the top of [the Python script](../python/badminton_landing_detector.py).

Windows PowerShell:

```powershell
py -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe python\badminton_landing_detector.py
```

macOS or Linux, on a compatible pylon/pypylon setup:

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python python/badminton_landing_detector.py
```

## C++ on Windows

Use a 64-bit Windows machine with:

- Visual Studio 2022 or its Build Tools with the **Desktop development with C++** workload and Windows SDK.
- CMake 3.20 or newer.
- Basler pylon Software Suite with the **C++ SDK** and camera transport installed. Use a release that ships `pylon-config.cmake`; see [Basler's CMake instructions](https://docs.baslerweb.com/pylonapi/cpp/pylon_programmingguide#building-applications-with-cmake-for-pylon).
- [OpenCV 4 for Windows](https://docs.opencv.org/4.x/d3/d52/tutorial_windows_install.html), built for x64 MSVC and with HighGUI enabled.

Set `CAMERA_A_SERIAL` and `CAMERA_B_SERIAL` in [the C++ source](../cpp/badminton_landing_detector.cpp) before compiling. In PowerShell, replace the sample OpenCV path with the directory containing your `OpenCVConfig.cmake`:

```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 "-DOpenCV_DIR=C:/opencv/build"
cmake --build build --config Release
.\build\Release\badminton_landing_detector.exe
```

If CMake cannot find pylon, add `"-Dpylon_DIR=C:/path/to/directory"` to the configure command, with the directory containing `pylon-config.cmake`. If the executable cannot load a DLL, add the actual OpenCV and pylon x64 runtime directories to the current shell's `PATH`. Use the same architecture for the compiler target and both SDKs. Rebuild after changing source constants.

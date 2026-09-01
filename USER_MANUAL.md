# Autonomous Coupling — User Manual

This guide takes a new installation from host setup to safe shutdown. It covers
the Ubuntu 24.04 / ROS 2 Jazzy reference system with a USB camera, MPU-9250,
Arduino Mega, and six encoded actuators.

> [!CAUTION]
> This software moves physical machinery. Read the safety section before
> energising an actuator. Keep a hardware emergency stop within reach.

## Contents

1. [Safety](#1-safety)
2. [Prepare the host](#2-prepare-the-host)
3. [Connect the hardware](#3-connect-the-hardware)
4. [Configure the installation](#4-configure-the-installation)
5. [Calibrate the system](#5-calibrate-the-system)
6. [Build and verify](#6-build-and-verify)
7. [Start the interface](#7-start-the-interface)
8. [Recover or home](#8-recover-or-home)
9. [Use acquisition mode](#9-use-acquisition-mode)
10. [Use manual mode](#10-use-manual-mode)
11. [Use automatic mode](#11-use-automatic-mode)
12. [Monitor and stop](#12-monitor-and-stop)
13. [Troubleshooting](#13-troubleshooting)
14. [Maintenance](#14-maintenance)

## 1. Safety

- Use a hardware emergency stop that removes actuator power independently of
  ROS, Python, and the Arduino.
- Secure the platform, camera, IMU, markers, cables, and mechanical end stops.
- Keep people and tools outside the full motion envelope while power is on.
- Test actuator direction, encoder polarity, scaling, and limits one actuator
  at a time before assembling all six.
- Start with low current limits and small commands.
- Never run two serial-owning controllers at once.
- Stop immediately on reversed feedback, binding, stale telemetry, or an
  unexpected movement.
- Keep `automatic_control.bypass_runtime_state: false` on hardware.

The **Confirm recovery at home** action changes software state; it does not
detect home. Use it only after mechanically verifying home and all six encoder
readings. Never use it merely to clear an error.

## 2. Prepare the host

### 2.1 Install dependencies

Install Ubuntu 24.04 and ROS 2 Jazzy using a suitable system image or the
official ROS procedure, then run:

```bash
sudo apt update
sudo apt install git python3-pip python3-colcon-common-extensions python3-smbus i2c-tools
sudo apt install ros-jazzy-rclpy ros-jazzy-std-msgs ros-jazzy-sensor-msgs ros-jazzy-cv-bridge
python3 -m pip install --user numpy pyyaml pyserial PySide6 matplotlib opencv-contrib-python imusensor
```

Verify the environment:

```bash
source /opt/ros/jazzy/setup.bash
ros2 --help
python3 -c "import cv2.aruco, serial, yaml, PySide6, matplotlib, rclpy, cv_bridge, smbus"
```

If the environment requires isolated Python dependencies, it must still see
the ROS 2 Python packages. Avoid incompatible OpenCV installations: both
`cv2.aruco` and `cv_bridge` must import successfully.

### 2.2 Obtain the repository

```bash
git clone <repository-url> Autonomous-Coupling
cd Autonomous-Coupling
```

Commands below run from this root. Do not copy generated `build/`, `install/`,
or `log/` directories between computers.

## 3. Connect the hardware

Keep actuator power disabled during communication checks.

### Camera

```bash
sudo apt install v4l-utils
v4l2-ctl --list-devices
ls -l /dev/video*
```

The default is camera index `0`, V4L2, 640 × 480, 30 FPS.

### IMU

Enable I2C, connect the MPU-9250, and scan bus 1:

```bash
i2cdetect -y 1
```

Runtime expects address `0x68` by default.

### Arduino and actuators

```bash
ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
sudo usermod -aG dialout "$USER"
```

Log out and back in after changing group membership. Upload
`src/stewart_control/arduino/pilotage_feedback_mp.ino` to an Arduino Mega 2560.
Confirm motor/encoder pins, direction, pulley diameter, counts per revolution,
and electrical limits against the assembly. Close Arduino Serial Monitor before
running ROS.

Serial uses newline-terminated CSV in centimetres:

```text
Host → Arduino: M1,M2,M3,M4,M5,M6
Arduino → Host: F1,F2,F3,F4,F5,F6
```

## 4. Configure the installation

Edit `src/stewart_control/config/stewart_params.yaml` and verify:

| Setting | Required check |
|---|---|
| `serial` | Actual Arduino path and firmware baud rate. |
| `stewart_platform` | Measured geometry, home pose, and nominal leg lengths. |
| `actuators` | Home vector and safe per-motor minimum/maximum travel. |
| `aruco` | Camera index, dictionary, marker ID, measured size, and reference. |
| `imu` | I2C address, calibration JSON, axis mapping, and home reference. |
| `fusion` | Timeouts and filter values; retain defaults for first commissioning. |
| `automatic_control` | Keep blind approach and runtime bypass disabled initially. |

For an untracked machine-specific configuration:

```bash
cp src/stewart_control/config/stewart_params.yaml /absolute/path/stewart_params.yaml
export STEWART_CONFIG=/absolute/path/stewart_params.yaml
```

Set that variable in every terminal that launches the system.

## 5. Calibrate the system

Recalibrate after changing a camera, resolution, marker, IMU mounting, actuator,
encoder, pulley, or platform geometry.

### 5.1 Camera

Stop other camera users, then run:

```bash
python3 src/stewart_control/calibration/calibrate_camera_charuco.py
python3 src/stewart_control/validation/validate_camera_calibration.py
```

The default ChArUco board is 6 × 9 squares with 30 mm squares and 24 mm
markers. Use `--help` for alternatives. Capture sharp views across the whole
frame at varied positions and angles. Outputs are stored under
`src/stewart_control/share/stewart_control/`.

At the defined aligned/home pose, establish the ArUco zero reference:

```bash
python3 src/stewart_control/test/test_single_marker_fixed_camera.py
```

Update `reference_position_m`, `reference_orientation_deg`, and axis signs/order.
Move one physical axis at a time and verify the software axis, sign, and scale.

### 5.2 IMU

The calibration utility defaults to sensors at `0x69` and `0x68`. For the
runtime sensor only:

```bash
python3 src/stewart_control/calibration/calibrate_dual_imu.py --only imu2
```

Follow the stationary gyro, six-face accelerometer, and magnetometer prompts.
Set `imu.calibration_path` to the generated JSON. Mount the IMU, place the full
platform at home, then establish its reference:

```bash
python3 src/stewart_control/test/test_imu_home_reference.py
```

### 5.3 Actuators

On a new host, complete the build in section 6 before opening this ROS
executable, then return here. With the platform unloaded or mechanically
supported:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run stewart_control motor_test_interface
```

For each actuator, verify direction, measure encoder distance with a ruler or
caliper, approach limits slowly, and save safe min/max values with margin. Stop
before a hard stop. Repeat the measurement before full assembly.

## 6. Build and verify

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select stewart_control
source install/setup.bash
ros2 pkg prefix stewart_control
ros2 pkg executables stewart_control
```

The prefix must point to this repository's `install` directory. Every new
terminal needs both `source` commands. Rebuild and re-source after changing
package data, launch files, or configuration.

### Preflight checklist

- [ ] Mechanics, cables, camera, markers, and IMU are secure.
- [ ] Motion envelope is clear and emergency stop is tested.
- [ ] Power/current limits match the test.
- [ ] Motor and encoder channels match M1–M6.
- [ ] Arduino, camera, and `0x68` IMU are present.
- [ ] No other process owns the serial port or camera.

With actuator power disabled, start acquisition:

```bash
ros2 launch stewart_control acquisition_launch.py
```

In a second sourced terminal:

```bash
ros2 node list
ros2 topic hz /camera/image_raw
ros2 topic hz /aruco_position
ros2 topic hz /imu_error
ros2 topic hz /imu_gyro
ros2 topic hz /F_orientation
ros2 topic echo /F_pose --once
```

Expect roughly 15–20 Hz camera preview and about 20 Hz IMU/fusion with current
defaults; pose rate depends on marker visibility. Stop with `Ctrl+C`.

## 7. Start the interface

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run stewart_control interface_node
```

The dashboard shows runtime status, mode actions, manual pose inputs, camera
preview, sensor/fusion values, actuator targets, feedback, health indicators,
and plots. Grey indicators mean a topic is stale; verify every stream required
by the selected mode.

## 8. Recover or home

A new installation or unclean shutdown blocks motion.

If the platform is already physically at home, disable power, verify the
mechanism and all six readings, then select **Confirm recovery at home**.

If position is uncertain, close all other motion nodes and perform controlled
homing with the emergency stop ready:

```bash
ros2 launch stewart_control launch_posHome.py
```

Homing sends the configured home targets, requires five stable reads within
0.3 cm, and times out after 30 seconds. Watch the mechanism, not only logs. Use
the dashboard recovery/calibration window for individual actuator recovery.

## 9. Use acquisition mode

1. Select **Acquisition**; it starts camera, IMU, and fusion without motors.
2. Confirm the correct marker, live image, and healthy sensor indicators.
3. At home, referenced ArUco and IMU values should be near zero.
4. Apply a known translation/tilt and verify axis, sign, scale, and fusion.
5. Return to home and wait for stable values before motion.

## 10. Use manual mode

Manual translation is in centimetres; roll, pitch, and yaw are in degrees.

1. Complete preflight and recovery, then energise at a safe limit.
2. Select **Manuel**. Inputs reset to zero and manual updates are enabled.
3. Send one small change on one axis.
4. Watch all six targets and feedback values; confirm direction and settling.
5. Repeat per axis before trying small combined poses.
6. Select **État initial** and verify all feedback returns home.

Out-of-range commands are rejected. Investigate the pose or mechanics rather
than widening limits without review.

## 11. Use automatic mode

1. Complete preflight, recovery, and an acquisition-only check.
2. Place the platform at verified home with stable marker visibility.
3. Energise actuators and keep the emergency stop ready.
4. Select **Automatique**. Acquisition starts if needed, followed by the
   automatic controller.
5. Introduce a small, controlled target offset.
6. Confirm command and feedback converge without approaching physical limits.
7. Verify new motion stops inside `stop_tolerance_cm`.
8. Emergency-stop on marker loss, stale feedback, or unexpected motion; recover
   before restarting.

Headless laboratory launch:

```bash
ros2 launch stewart_control stewart_ordered_launch.py
```

This starts IMU, ArUco, fusion, and automatic control, but no GUI or staged
startup delays.

## 12. Monitor and stop

Useful live checks:

```bash
ros2 topic hz /F_pose
ros2 topic hz /feedback_motors
ros2 topic echo /stewart_status
python3 scripts/performance_monitor.py --duration 120
```

The monitor writes CSV and JSON results under `performance_logs/`. Review rates,
staleness, CPU/memory, payload size, and command-to-feedback latency before
tuning.

The safety state is stored in
`~/.stewart_control/motor_runtime_state.json`. Do not edit/delete it to bypass
recovery; use `STEWART_RUNTIME_DIR` to relocate it before launch.

### Safe shutdown

1. Make the target stationary and stop automatic movement.
2. Select **État initial** and wait for six home feedback values.
3. Use the dashboard stop action and wait for child processes to exit.
4. Close the GUI or press `Ctrl+C` in its terminal.
5. Confirm clean-shutdown/home messages, then remove actuator power.

Motion nodes attempt to park at home during graceful shutdown. After a crash,
timeout, emergency stop, or unconfirmed park, expect recovery next time.

## 13. Troubleshooting

| Symptom | Checks and action |
|---|---|
| Package not found / stale code | Rebuild, re-source both setups, and check `ros2 pkg prefix stewart_control`. |
| Serial missing/denied | Check `/dev/ttyACM*`, cable, `dialout` membership, config, and stable `/dev/serial/by-id` paths. |
| Serial busy | Stop every other automatic, manual, home, motor-test, and Arduino-monitor process. |
| Motion disabled | Perform verified recovery; do not enable runtime-state bypass. |
| Camera unavailable | Stop other camera apps; check `/dev/video*`, index/backend, and run `test_camera_feed.py`. |
| Marker absent/wrong | Check dictionary, ID, physical size, focus, exposure, lighting, calibration, and `test_detect_aruco_ids.py`. |
| IMU absent/noisy | Check `i2cdetect -y 1`, address, wiring, calibration, mounting, and `test_imu_sensor.py`. |
| Yaw unstable | Move IMU/wiring away from motors, steel, and high-current conductors; recalibrate magnetometer. |
| Commands but no motion | Check status topic, recovery, limits, serial ownership, Arduino power, E-stop, and baud rate. |
| Wrong direction/scale | Stop immediately; correct motor/encoder mapping, polarity, CPR, or pulley geometry and recalibrate. |
| GUI fails | Verify graphical session and `python3 -c "import PySide6, matplotlib, cv_bridge"`. |

For sensor verification:

```bash
python3 scripts/verify_sensor_calibration.py
python3 src/stewart_control/validation/validate_camera_calibration.py
```

Tune only after raw inputs are correct. Change one fusion/control parameter at
a time and capture a performance baseline before and after.

## 14. Maintenance

Before each run, inspect mechanics and wiring, verify the emergency stop and
device paths, check recovery state, and validate acquisition before motion.

After hardware changes, re-measure geometry/travel, recalibrate affected
devices, re-establish references, and repeat low-power manual commissioning.

Before a release:

- build from a clean shell and exercise all documented workflows;
- review calibration artifacts for machine-specific or sensitive data;
- align `package.xml`, `setup.py`, and `LICENSE` metadata;
- exclude generated build/log/performance output;
- record tested hardware, configuration, and firmware revisions.

See the [project README](README.md) for architecture, topics, configuration, and
repository structure.

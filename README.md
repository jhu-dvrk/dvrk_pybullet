# dvrk_pybullet

PyBullet implementation of the backend contracts defined by
`dvrk_simulator_base`. The package will load robot assets from `dvrk_model` and
must not depend on SurRoL.

PyBullet is required only when the backend runs, not when it builds.  Launch
files use the current Python interpreter when it can import `pybullet`.  If
PyBullet is installed in a different virtual environment, select it for the
current shell without changing project files:

```shell
export DVRK_PYBULLET_PYTHON="$HOME/wss/dvrk/.venv/bin/python"
```

The selected interpreter is then recorded in
`<workspace>/.generated/pybullet/python-runtime.json`, so later shells reuse
it without the environment variable.  The launch log reports both the chosen
interpreter and whether it came from the environment, saved selection, or
current ROS Python.  Removing `.generated` simply causes the next launch to
select it again.

The simulator currently supports shared-world kinematic PSM and ECM models,
CRTK ROS interfaces, and an ECM optical camera exported through GStreamer's
Unix-FD transport.

## Model generation cache

`dvrk_model` is located only through the ROS 2 ament index. The Virtual PSM
Xacro is expanded and its `package://dvrk_model/...` resources are converted to
validated absolute paths for PyBullet. Generated URDF and metadata files live
outside `src`:

```text
<workspace>/.generated/pybullet/<content-hash>/
├── model.urdf
└── metadata.json
```

The content-addressed entry is reused while the expanded model, instrument,
parent link, and materializer version remain unchanged.

## Preview PSM1

Build and source the two packages from the workspace, activate the environment
containing PyBullet, and open the GUI:

```shell
cd ~/wss/dvrk
colcon build --symlink-install \
  --packages-select dvrk_simulator_base dvrk_pybullet
source install/setup.bash
ros2 run dvrk_pybullet dvrk_pybullet_preview --model PSM1 --instrument 420006
```

The preview loads the Virtual PSM at its configured home position and remains
open until the window is closed or Ctrl-C is pressed. Use `--duration 10` for
an automatically closing ten-second preview.

## ROS simulator node

Start a configured scene and expose its CRTK ROS graph:

```shell
ros2 launch dvrk_pybullet simulator.launch.py \
  scene:=ECM_PSM1_PSM2.yaml
```

The only optional launch argument is `config:=/path/to/pybullet.yaml`. Runtime
settings, including GUI, rates, queue capacity, renderer, and generated asset
location, belong in that backend configuration. Robots, instruments, camera,
and transport settings belong in the scene YAML.

## ECM camera

Camera settings live in each scene's `camera` mapping and use the same core
field names as `dvrk_isaac_sim`: `mode`, `owner`, `frame`, `width`, `height`,
`horizontal_fov_deg`, `near_clip_m`, `far_clip_m`, `encoding`,
`baseline_m`, `publish_rate_hz`, and `transports`. PyBullet supports mono and
side-by-side stereo `rgba8` output and adds this transport-specific section:

```yaml
transports: [unixfd]
unixfd: {socket_path: "@dvrk:pybullet:mono_source"}
```

Start a scene containing an ECM, then connect a GStreamer viewer from another
terminal:

```shell
ros2 launch dvrk_pybullet simulator.launch.py scene:=ECM_PSM1_PSM2.yaml

gst-launch-1.0 unixfdsrc socket-path=dvrk:pybullet:mono_source \
  socket-type=abstract \
  ! queue leaky=downstream max-size-buffers=1 \
  ! videoconvert ! autovideosink sync=false
```

The socket is named `@dvrk:<package>:<stream>`, identifying `pybullet` as the
producer and `mono_source` as the stream. A scene with `mode: stereo` uses
`@dvrk:pybullet:stereo_source`; `width` and `height` remain per-eye dimensions.
Both endpoints use the `dvrk_data` abstract socket notation. The producer uses
one Linux `memfd` per frame and a one-frame leaky queue, so a
slow or disconnected viewer cannot build an image backlog. Headless operation
uses PyBullet's EGL renderer. Set `renderer: tiny` in `pybullet.yaml` for a
CPU-rendered diagnostic run.

The [`share/open-xr`](share/open-xr) configuration connects the three-PSM
patient-cart scene to a Quest surgeon console through `sawOpenXR`. It includes
the `dvrk_system` JSON, the low-latency Unix-FD GStreamer input, and startup
instructions. Once the separately built optional `saw_openxr` package is
sourced, start the complete setup with:

```bash
ros2 launch dvrk_pybullet open_xr.launch.py
```

`ECM_PSM1_PSM2_PSM3.yaml` adds PSM3. Scene files select each robot asset and
set non-overlapping world base poses. All arms have independent command
mailboxes and CRTK interfaces, while the simulation itself advances once per
world tick.

The Virtual ECM is always controlled kinematically; no mass, motor, or PID
tuning is used. When it is present, each PSM's top-level Cartesian state and
commands use the live `ECM_view` frame. The corresponding
`/<PSM>/local/measured_cp` and `/<PSM>/local/setpoint_cp` topics remain in the
fixed PSM base frame. An explicitly `world`-framed Cartesian command remains
available for diagnostics.

`dvrk_arm_test.py` includes the selected arm in its ROS node name, so tests for
different arms can run concurrently without a manual node-name remap:

```shell
ros2 run dvrk_python dvrk_arm_test.py -a PSM1

ros2 run dvrk_python dvrk_arm_test.py -a PSM2
```

Inspect the state from another sourced terminal:

```shell
ros2 topic list | grep PSM1
ros2 topic echo /PSM1/measured_js --once
ros2 topic echo /PSM1/measured_cp --once
```

Send one direct joint setpoint (positions are radians except insertion, which is
metres):

```shell
ros2 topic pub --once /PSM1/servo_jp sensor_msgs/msg/JointState \
  "{name: [yaw, pitch, insertion, roll, wrist_pitch, wrist_yaw], position: [0.2, 0.1, 0.14, 0.0, 0.2, -0.2]}"
```

Send a velocity-limited joint move and open the jaw:

```shell
ros2 topic pub --once /PSM1/move_jp sensor_msgs/msg/JointState \
  "{name: [yaw, pitch, insertion, roll, wrist_pitch, wrist_yaw], position: [-0.2, 0.15, 0.10, 0.3, -0.2, 0.2]}"

ros2 topic pub --once /PSM1/jaw/move_jp sensor_msgs/msg/JointState \
  "{position: [0.7]}"
```

Cartesian servo and move commands use world-frame poses when running a single
arm without an ECM reference:

```shell
ros2 topic pub --once /PSM1/servo_cp geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: 0.01, y: 0.0, z: -0.1237}, orientation: {x: 0.7071, y: 0.7071, z: 0.0, w: 0.0}}}"
```

The PyBullet backend solves all six pose coordinates numerically using its own
forward kinematics. This keeps the URDF mimic joints constrained by the logical
PSM joints during IK.

Motion is accepted while the arm is enabled and homed. The node starts in that
state. Operating-state commands can be tested with:

```shell
ros2 topic pub --once /PSM1/state_command crtk_msgs/msg/StringStamped \
  "{string: pause}"
ros2 topic pub --once /PSM1/state_command crtk_msgs/msg/StringStamped \
  "{string: resume}"
```

`servo_jp` commands supersede older pending servo commands; `move_jp` and state
commands use a bounded ordered queue. Joint and jaw limits are checked before a
command is applied. Moves are synchronized linear trajectories using the
configured velocity limits, and the operating-state `is_busy` flag covers the
move. The jaw command drives the URDF's named mimic joints.

Control is deliberately kinematic at this milestone: setpoints are applied with
PyBullet joint resets. No link masses, motor gains, or PID tuning are required
until the backend advances to dynamic control. ROS callbacks only validate and
enqueue commands; all PyBullet calls remain on the owner thread.

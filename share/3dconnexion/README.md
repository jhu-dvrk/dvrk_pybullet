# PyBullet patient cart with two 3Dconnexion MTMs

This example uses two `mts3Dconnexion` components as non-haptic MTMs and the
PyBullet ROS interfaces as the patient cart. MTMR controls PSM1, while MTML
controls both PSM2 and PSM3. The console also provides bimanual MTML/MTMR ECM
teleoperation.

Both supplied device configurations match the two detected SpaceNavigator
devices (`046d:c626`). Since these devices do not expose unique serials, MTML
is explicitly bound to `/dev/hidraw6` and MTMR to `/dev/hidraw7`. Update each
configuration's `path` if Linux assigns different hidraw numbers after a reboot
or reconnect.

PSM2 and PSM3 share MTML but are mutually exclusive console selections. PSM2
is selected initially; select `MTML_PSM3` from the dVRK console or rqt console
to transfer MTML control to PSM3. Selecting either mapping automatically
unselects the other.

Each device JSON places its MTM in the user workspace for ECM teleoperation
while preserving the SpaceNavigator orientation mapping. MTML is translated to
`[0.18, 0.40, 0.475]` metres and MTMR to `[-0.18, 0.40, 0.475]` metres.

After building and sourcing the workspace, launch the complete system with:

```bash
source ~/wss/dvrk/.venv-pybullet/bin/activate
source ~/wss/dvrk/install/setup.bash
ros2 launch dvrk_pybullet 3dconnexion.launch.py
```

The launch file starts PyBullet, `dvrk_system`, the helper that enables the
console, and a GStreamer preview of the virtual stereo camera. Disable the
preview with `preview:=false`. Use `scene:=peg_board_ring.yaml`, `headless:=false`,
or `rqt:=true` as needed.

To open the preview manually, run:

```bash
gst-launch-1.0 unixfdsrc \
  socket-path=dvrk:simulator:stereo_source socket-type=abstract \
  do-timestamp=true \
  ! queue leaky=downstream max-size-buffers=1 \
  ! videoconvert ! autovideosink sync=false
```

Linux users must also grant their account access to both corresponding
`/dev/hidraw*` devices as described in the `saw3Dconnexion` README.

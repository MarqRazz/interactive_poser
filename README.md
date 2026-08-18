# interactive_poser

Measure where something in a robot cell actually is, by hand, in RViz, against
live sensor data.

A workcell is drawn in a URDF and then bolted together by a person. The two
disagree: a camera sits a few millimetres off its bracket, a fixture is bolted a
centimetre from where the drawing puts it. This tool lets an operator drag the
model until it matches what the sensors see, and writes the difference to a YAML
file the URDF loads.

## What it produces

A file of offsets from nominal, in metres and radians. All zeros means built as
drawn, so every number in the file is build error:

```yaml
calibration:
  joints:
    front_camera_mount_joint:
      cloud_topics: [/front_camera/depth_registered/points]
      image_topic: /front_camera/color/image_raw
      camera_info_topic: /front_camera/color/camera_info
      x: 0.0041
      y: -0.0122
      z: 0.0037
      roll: 0.0031
      pitch: 0.0008
      yaw: -0.0094
```

Read that as: the camera is 12.2 mm out in y and rotated half a degree in yaw.
The same file feeds the URDF and tells the tool which joints it may target, so
the two cannot disagree.

## Where zero error sits

The tool edits one fixed joint at a time and its limits, about 1 m and 15
degrees, apply to how far that joint sits from **nominal**. It needs to know
where nominal is. There are two ways to tell it.

### Give the device a mount link (recommended)

Put the layout geometry in a `*_mount_link` above the calibration joint, so that
joint's origin is zero on a cell built as drawn:

```
camera_post_link
  └── camera_post_mount_link           nominal, from the layout drawing
        └── [calibration joint]        all zeros until measured
              └── front_camera_link    as built
```

Many devices already sit this way and need nothing extra: bolted to a mount face,
a tool plate on a flange, a sensor on its bracket.

### Or declare the nominal in the file

Where the layout is baked into the joint itself, say what nominal is:

```yaml
fixture_joint:
  nominal: {x: 0.0, y: 0.65, z: 0.0, roll: 0.0, pitch: 0.0, yaw: -1.5707963}
  x: 0.0
  y: 0.65
  z: 0.0
  roll: 0.0
  pitch: 0.0
  yaw: -1.5707963
```

`x`/`y`/`z`/`rpy` stay the absolute joint origin, which is what the xacro
applies. `nominal` tells the tool where zero error is, so the limits, the marker
parameters and the log all read as error.

Two costs. The nominal is now stated in the xacro and here, and the two can
drift. And composition becomes visible: the offset applies in the device's own
frame, `absolute = nominal * offset`, so under a -90 degree nominal an offset in
the device's Y lands along the parent's X.

Declare neither and the tool refuses the joint rather than clamping it.
[docs/design.md](docs/design.md) explains why the offsets are errors rather than
absolute origins, and what clamping the wrong quantity would destroy.

## Scope

Fixed transforms between rigid bodies a human positioned: sensor mounts,
fixtures, tool plates. The tool reads each target's parent, child and type from
`/robot_description` and refuses anything that is not a fixed joint.

It does not calibrate joint values. Axis zero points belong to the robot
controller, a fraction of a degree is not visible by eye, and identifying one
properly takes many poses and a least-squares fit. That is a different tool.

Accuracy is whatever a person can achieve by eye against a noisy depth cloud.
Good enough to catch a fixture bolted two centimetres off. Not a substitute for
target-based calibration when you need better.

## Build

A normal ament_cmake package. Drop it into a workspace alongside the cell you
are calibrating:

```bash
colcon build --symlink-install --packages-select interactive_poser
source install/setup.bash

colcon test --packages-select interactive_poser && colcon test-result --verbose
```

`colcon test` runs `ament_lint_auto` only. There are no unit tests: exercising
this package means driving a cell, which it deliberately does not ship.

## Calibrating a cell

Work in this order. A fixture has no sensor of its own, so it can only be aligned
against a camera you already trust.

**1. Decide what is calibratable** and list those joints in the calibration file,
all zeros. Give each one a nominal, per "Where zero error sits" above.

**2. Bring the cell up against that file** and check the tool agrees with it:

```bash
ros2 launch interactive_poser calibrate.launch.xml \
     calibration_file:=/path/to/your_calibration.yaml
```

A cell package is a better home for that path than your shell history. Wrap this
launch in one of your own that fills in `calibration_file`, and `output_directory`
if saves should land somewhere other than beside it:

```xml
<include file="$(find-pkg-share interactive_poser)/launch/calibrate.launch.xml">
  <arg name="calibration_file"
       value="$(find-pkg-share my_cell_config)/config/default_calibration.yaml"/>
</include>
```

If the node reports a joint reading differently from the file, the cell was
launched without the `calibration_file` argument. Fix that before measuring
anything, or you will record an offset against the wrong baseline.

**3. Calibrate each camera against something you trust.** The arm is usually the
best target in the cell: it is a precision machine, its model is accurate, and
you can drive it into view. Move it somewhere well inside the camera's frame,
then drag the camera's marker until the relayed cloud sits on the arm's meshes.
Use the camera overlay for rotation, where mesh silhouettes against real image
edges resolve angle far better than a cloud does.

**4. Calibrate the fixtures against the cameras.** Switch target from the marker
menu. The fixture's ghost is what moves now; align it onto the point cloud of the
real fixture.

**5. Save once at the end.** Edits are held per joint, so one save records every
joint you touched. Reload the cell against the saved file and confirm the change
took.

### Knowing when you are done

Alignment by eye against a noisy depth cloud is worth a few millimetres, not
tenths. Two things help. Sight along edges and corners rather than flat faces: a
flat deck or tabletop constrains almost nothing about rotation. And check a second
view before accepting, because an alignment that looks perfect down one axis is
often several millimetres out along the camera's line of sight.

To find out what your own alignment is worth, set a known offset with
`ros2 param set`, realign by eye, and read the residual. That number is your
practical accuracy, and it is more honest than anything this document can claim.

## Running it

Bring up your cell first, passing it the calibration file so the description and
the tool agree. Then:

```bash
ros2 launch interactive_poser calibrate.launch.xml \
     calibration_file:=/path/to/your_calibration.yaml
```

The node lists every joint it found in the file, refuses any that is not a fixed
joint, and starts on the first. Pass `target:=<joint_name>` to start elsewhere,
or switch from the marker's right-click menu. `rqt_reconfigure` has no enum
editor, so the menu is the better route; `target_calibration_joint` exists for
launch arguments and scripts.

Drag the marker for coarse placement and type exact values for the last
millimetre:

```bash
ros2 param set /interactive_poser offset.x_mm 12.35
ros2 param set /interactive_poser offset.yaw_deg -0.54
```

Parameters are millimetres and degrees and quantise to two decimals. Their
bounds double as a sanity threshold, so the tool cannot emit an absurd offset.

### Saving

Save from the marker menu or `ros2 service call /interactive_poser/save
std_srvs/srv/Trigger`. Saves are timestamped
(`calibration_20260818T140413Z.yaml`) and never overwrite the input, so
adopting a result is a deliberate copy. Reload the cell against the new file to
apply it. Restart `robot_state_publisher` and `move_group`; the driver and
controllers do not need restarting, because no arm joint changed.

A save records **every** target, read from the live system: the joint under your
hand from the marker, any joint you edited earlier this session from that edit,
and the rest straight from TF. The file therefore describes the cell as it is
actually running. Reload from nominal and save, and you correctly get nominal
back rather than a stale earlier result. Earlier saves stay on disk.

Edits are held per joint for the session, so switching target and coming back
keeps your work, and one save records every joint you touched.

## What you see

`config/calibration.rviz` has a display for each output. The names are fixed, so
switching target never means re-pointing RViz.

| Output | Contents |
|---|---|
| `/interactive_poser/snapshot` | the posed point cloud |
| `/interactive_poser/image` | the posed colour image |
| `/interactive_poser/camera_info` | its intrinsics, in the posed frame |
| `/interactive_poser/ghost` | the target's own meshes, in the posed frame |

For a camera, drag until the cloud sits on the model, using the overlay for
rotation. For a fixture there is no cloud of its own, so drag until the ghost
sits on the cloud from an already-calibrated camera.

### Frozen and live

The relay holds a frozen snapshot by default: steady to align against. Switch to
Live relay from the marker menu, or with
`ros2 param set /interactive_poser live_relay true`, when the scene itself is
changing. Streaming is decimated by `stream_decimation`; switching back off
freezes the frame you were looking at, at full resolution.

### Ghost appearance

```bash
ros2 param set /interactive_poser ghost_color "[0.1, 0.9, 0.9]"
ros2 param set /interactive_poser ghost_alpha 0.35
ros2 param set /interactive_poser ghost_alpha 0.0     # hide it
```

Pick a colour absent from the scene, so a mismatch stands out. Hiding the ghost
suits a camera target, where the point cloud is the better cue.

## Troubleshooting

**The camera view stays blank, reporting `No CameraInfo received`.** Raise
`image_refresh_period`, or check it is not 0. The default 2 s heartbeat exists to
close a startup race in RViz's Camera display.

**The cloud does not follow the marker.** Raise `settle_time`, so the
full-resolution cloud is not sent before you have stopped moving. Marker geometry,
including the ghost, tracks without this.

**A joint is refused at startup.** Either it is not a fixed joint, or its origin
is too far from nominal to be build error. See "Where zero error sits".

**A joint reads differently from the file at startup.** See step 2 of
"Calibrating a cell": the cell and the tool are not on the same file.

## How it works

[docs/design.md](docs/design.md) explains the design: what the offsets mean, why
the shadow frames are static, why most of the code is about getting RViz to
redraw, and what was rejected along the way.

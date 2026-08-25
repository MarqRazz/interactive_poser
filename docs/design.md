# Design

Why interactive_poser is built the way it is. Read
[the README](../README.md) first for what it does and how to use it, and
[xacro-integration.md](xacro-integration.md) for the consumer side.

## The shape

A hand-driven editor, not a solver. The operator supplies the judgement; the tool
supplies a way to move geometry, see the effect, and write the answer somewhere
the description will read it. A target-based calibration would be more accurate
and needs a fixture, a capture routine and a solver; this fills the gap below
that.

## Offsets are error, not geometry

Every value the tool edits is an offset from the target's **nominal** pose. The
absolute origin is only ever a derived quantity, computed at the last moment when
the file is written.

This is the decision the rest of the design hangs off. It buys four things.

A file of errors is readable. `y: -0.0122` says the part is 12 mm out. An
absolute `y: 0.6378` says nothing without knowing what it should have been.

Errors are comparable across instances of the same cell design, because both are
expressed against the same drawing rather than against their own geometry.

Limits become meaningful. The tool clamps to roughly a metre and fifteen degrees
because that is the plausible range of a build error. Applied to an absolute
origin those numbers are arbitrary, and worse, they would clamp real layout: a
joint mounted at -90 degrees would seed as -15 and quietly destroy the cell's
geometry on save.

And the marker needs no arithmetic. It lives in the target joint's parent frame,
so under the usual convention (nominal at identity) the marker's pose *is* the
offset. There is no composition step to get backwards.

Nominal is identity by default, which is why the README recommends putting layout
geometry above the calibration joint in a `*_mount_link`. Where that is not
practical the file can declare `nominal` explicitly, and the tool works relative
to that instead. The cost of the second form is that the layout is then stated in
two places, and that composition becomes visible: `absolute = nominal * offset`,
so an offset is expressed in the device's own frame.

## The file is the schema

`calibration.joints` is both the offsets the URDF applies and the list of joints
the tool may edit. One file, two readers, so the tool cannot offer a target the
description ignores.

Each entry carries only a joint name and its numbers. Parent, child and joint
type are read from `/robot_description` rather than restated in config. Restating
them would invite the two to drift, and reading them is what lets the tool
enforce its own scope: anything that is not a fixed joint is refused, because a
joint value belongs to the robot controller and is not observable by eye anyway.

A joint is also refused when its origin sits further from nominal than a build
error could plausibly be. That check exists because the alternative is silent
corruption rather than a loud failure.

## Shadow frames

Editing shadow frames rather than the real tree means nothing downstream sees a
change until a saved file is loaded, so the tool is safe to run against a live
system.

The shadow transforms go out on `/tf_static`, which is deliberate rather than
convenient. A static transform is timeless, so a snapshot stamped minutes ago
still resolves against the newest offset. Broadcast dynamically at the current
time, a frozen cloud stamped in the past would never move with the marker.

Which frame the sensor data actually rides is learnt from the incoming message,
not assumed from the joint. A colour cloud is often published in the colour
optical frame; bridging a guessed frame produces a shadow nothing is ever
published into, and a display that stays silently empty.

## Making the display move

Most of the tool's complexity is not calibration mathematics. It is getting RViz
to redraw.

RViz does not re-transform a point cloud it already holds when that cloud's
transform moves. A cloud is positioned when it arrives and then left alone. So
tracking the marker means resending the cloud, and at 1280x720 that is about
29.5 MB a message: roughly 400 MB/s at sensor rate. That one figure is behind the
frozen default, the decimated drag preview, and the decimation on live mode.

The compromise is two resolutions. While the offset is changing the tool resends
a decimated copy, gated on the pose having actually moved and throttled. Once
things go quiet a settle timer resends the full cloud. Neither a marker drag nor
an rqt slider announces that it has finished, so a debounce is the only signal
available.

Markers behave better. `Marker` carries `frame_locked`, which asks RViz to
re-transform it every update cycle, so the ghost follows the handle with nothing
republished. `InteractiveMarker` has no such field and needs none, because RViz
re-resolves its header frame anyway. `PointCloud2` has no equivalent at all,
which is the whole reason the resend path exists.

The camera overlay has its own trap. RViz's Camera display renders on the *image*
callback and pairs the image with whatever CameraInfo it already holds. An image
published once can arrive before the first CameraInfo, get discarded, and never
be retried because no second image follows. The symptom is a permanently blank
view blaming the info topic. A slow heartbeat resending the frozen image removes
the race.

Output topics are created once and outlive a target switch. Per-target names
would force the operator to re-point RViz every time they changed joint, which is
exactly the friction the fixed names avoid.

## Why cameras come first

Moving data and moving a model are not symmetric. A sensor carries its own
evidence; a fixture has none, so its ghost can only be judged against data from a
camera that is already trusted. The calibration order falls out of that, and is
not a preference.

The ghost walks the URDF subtree below the target and stops at any other
calibration joint, because anything past one is posed independently and should
not be dragged along by its parent's measurement.

## Keeping one number

The stored transform, the ROS parameters and the saved file must agree. Two
separate bugs came from clamping or rounding one and not the others, and the
result each time was a GUI showing a value the file did not contain.

Parameters carry millimetres and degrees, quantised to two decimals. The
quantisation is not cosmetic: `rqt_reconfigure` renders its text box from the
value itself and ignores the descriptor's step, so the only way to show two
decimals is for the value to have two decimals. Rounding happens in a pre-set
callback, before the value is stored, because rounding afterwards would mean
calling `set_parameters` from inside a parameter callback.

Clamping limits magnitudes, not axes. The marker's `MOVE_AXIS` controls inherit
its orientation, so once the handle is rotated a drag along its local Y has
components in several parent axes. Clamping those independently lets the
unsaturated ones keep growing after one hits the stop, and the marker shears
sideways instead of stopping. Rotation is slerped back along the same axis for
the same reason.

## Why a save reads the live system

TF already knows where every calibrated joint sits, so there is no history to
carry. Reading the previous save instead would produce a document mixing what is
loaded with what was once measured, describing a cell that does not exist.

That is why the file is a snapshot of the running system rather than an
accumulation, and why reloading from nominal and saving correctly yields
nominal. In-session edits are the one exception, held per joint so that switching
target does not silently discard work.

## Rejected

**A runtime TF override.** It would apply instantly with no restart. It also
moves only the frames: MoveIt would keep planning against collision geometry at
nominal. That is a cosmetic fix dressed as a real one, which is worse than no fix
on a cell where an arm works near bolted steel.

**Storing absolute kinematics, as UR does.** It works, and the tool supports it
through a declared `nominal`, but it makes the common case unreadable. The
mount-link form is preferred wherever the description can be shaped for it.

**Calibrating joint values.** Axis zero points live in the robot controller, are
not visible by eye, and need many poses and a solver to identify. Different tool.

**Republishing at a fixed rate**, which the original prototype did at 10 Hz with
the full cloud. It tracks perfectly and costs about 300 MB/s. The decimated
preview plus settle timer gets the same result for a fraction of it.

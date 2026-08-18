# Loading a calibration into the URDF

The calibration file is only useful once the description applies it. This is the
consumer side: what a cell's xacro has to do so `robot_state_publisher`, MoveIt,
the planning scene and RViz all agree on where things are.

Putting the offsets into the description, rather than overriding TF at runtime,
is deliberate. TF is not the only consumer. A runtime override moves the frames
and leaves the collision geometry MoveIt plans against sitting at nominal, which
is a cosmetic fix wearing the costume of a real one.

## Give each calibratable device a mount link

The README's "Where zero error sits" covers when a device needs this and when it
already has it. This is the xacro shape it asks for: split the joint in two, with
the nominal half above and the calibration half below.

```xml
<!-- Where the drawing puts the sensor. -->
<link name="${prefix}camera_post_mount_link"/>
<joint name="${prefix}camera_post_mount_nominal_joint" type="fixed">
  <parent link="${prefix}camera_post_link"/>
  <child  link="${prefix}camera_post_mount_link"/>
  <origin xyz="0.71 0 1.336" rpy="0 ${pi/2} 0"/>
</joint>
```

Anything that must not move with the device's calibration parents to the mount
link rather than to the device. A pole bolted to the floor beside a fixture is
the usual case: hang it off the nominal frame and calibrating the fixture cannot
drag it.

## Load the file and apply the offsets

```xml
<xacro:arg name="calibration_file" default=""/>
<xacro:property name="calibration_file" value="$(arg calibration_file)"/>
<xacro:property name="calibration"
                value="${(xacro.load_yaml(calibration_file).get('calibration') or dict()).get('joints') or dict() if calibration_file else dict()}"/>

<xacro:property name="cal_cam"
                value="${calibration.get(prefix + 'front_camera_mount_joint') or dict()}"/>
<joint name="${prefix}front_camera_mount_joint" type="fixed">
  <parent link="${prefix}camera_post_mount_link"/>
  <child  link="${prefix}front_camera_link"/>
  <origin xyz="${cal_cam.get('x', 0)} ${cal_cam.get('y', 0)} ${cal_cam.get('z', 0)}"
          rpy="${cal_cam.get('roll', 0)} ${cal_cam.get('pitch', 0)} ${cal_cam.get('yaw', 0)}"/>
</joint>
```

Every lookup uses `.get(..., 0)`, so a joint absent from the file, or no file at
all, yields an identity origin instead of an error.

Use `dict()` and never a literal `{}`. Xacro's `${}` substitution is not
brace-balanced, so a `{}` inside an expression closes it early and the parse
fails with `'{' was never closed`.

Where a device comes from a third-party macro you do not own, the same values go
into the `<origin>` block passed at the call site.

## Pass the argument through every launch path

Each launch file that builds a `robot_description` needs the argument, including
the simulation one. A cell brought up without it runs at nominal while the tool
believes the file is applied. The tool warns when it detects that, but the warning
only fires if a target's file value and its live transform disagree.

## Check it

Two properties are worth testing rather than assuming.

An empty or absent calibration file must reproduce the nominal cell exactly.
Compare the generated URDF before and after the change, joint by joint. Note
that a file of explicit zeros produces `0.0` where an absent one produces `0`,
so compare parsed values, not text.

A measured file must change only the joints it names. Compose world poses for
every link and confirm nothing else moved.

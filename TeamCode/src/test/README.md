# Desktop tests and OpMode harness

These run on your computer's JVM in a few seconds. No robot, no Control Hub,
no phone, no emulator.

## Running them

- **Android Studio**: click the green arrow next to any test class or method,
  or right-click the `test` folder and choose Run.
- **Command line**: `gradlew :TeamCode:testDebugUnitTest`
  (if Gradle complains about the JVM version, point it at JDK 21:
  `JAVA_HOME=C:\Users\<you>\.jdks\jbr-21.0.11 gradlew :TeamCode:testDebugUnitTest`)

An HTML report lands in `TeamCode/build/reports/tests/testDebugUnitTest/index.html`.

## How real OpModes run here

The OpModes in TeamCode run **unmodified**. `OpModeRunner` gives one:

- a `hardwareMap` whose `get(type, name)` returns a stand-in motor registered
  under that configuration name (`DesktopHardwareMap`). A name that is not
  there throws the SDK's own `IllegalArgumentException`, so code that catches
  it to carry on without a device takes the same path as on the robot;
- telemetry that refreshes the way a Driver Station does, at most every 250 ms;
- `gamepad1` and `gamepad2`, fed from `DriverStation` or a real pad;
- a simulated clock: every `ElapsedTime` the OpMode creates reads it, so a
  30-second autonomous runs in milliseconds in a test, and in real time in the
  window.

It then calls `init`, `init_loop`, `start`, `loop` and `stop` in the order the
SDK's `OpMode.internalRunOpMode` does, with a `telemetry.update()` after each
one except `init()`. It stops an `@Autonomous` after 30 seconds, then sets every
motor to zero, as the robot controller's default OpMode does. If the OpMode
throws, it stops, and the exception is kept and shown.

`LinearOpMode` is not supported: it runs on its own thread.

`AutoRunTest` drives HiveKnightAuto (`BIOBUZZAuto`) this way. A few of its
tests pin behaviour that is almost certainly a bug, and say so; flip them when
the auto is fixed.

## Watching it from a gamepad

Plug a controller into the PC (or pair it over Bluetooth), then:

```
gradlew :TeamCode:gamepadGui        # field view
gradlew :TeamCode:gamepadHarness    # console dashboard
```

It opens in **INIT**, so the pad makes the OpMode's own choices. For
HiveKnightAuto, first A (Hive side) or B (Wall side), then A Leave, B Park,
X Launch or Y LaunchAndPark; the right bumper goes back to choosing a side.
Then **START** on the pad, or **S** in the window, runs it. **R** restarts it in
INIT. BACK + START, Esc, or closing the window quits.

To test a different OpMode, press **O** or click the OpMode name at the bottom
right of the window. The menu lists what the Driver Station would: every
`@TeleOp` (drive it with the pad) and `@Autonomous` OpMode in TeamCode, found
automatically. Picking one loads it fresh in INIT. A `LinearOpMode` is listed
but greyed out, since the harness only runs iterative OpModes. An OpMode that
needs something the stand-in robot lacks (such as the turret motor) shows as
crashed, with the line that asked for it.

There is a `Webcam 1`, but OpenCV cannot run on the PC, so `SimCamera` stands in
for the AprilTag processor, and `SimVision` works out what it would report from
the robot's pose: one `AprilTagClusterDetection` per CELL in view, posed at the
CELL opening the way the SDK's BIOBUZZ library defines it, with `ftcPose`
converted by the SDK's own formula. The camera sits where the OpMode's
`setCameraPose` puts it, read the SDK's way (yaw/pitch/roll 0 looks straight up,
pitch -90 looks forward); its view is drawn on the field and tags it sees turn
green. Field of view and range are assumptions (`SimVision`). The `VisionPortal`
itself is null, since its class cannot even be loaded off the robot.

To start on a particular OpMode, pass its class name:
`gradlew :TeamCode:gamepadGui --args="--gui --opmode=SomeOpMode"`.

The window shows the robot dead reckoned on the BIOBUZZ field, the ball path
(intake, ramp, flywheels), the pad, the motor powers, and the telemetry a Driver
Station would show. A crash is shown with the line in the OpMode that threw.

The field (`BiobuzzField`, `FieldSim`) follows Competition Manual Sections 9-11,
the Event Field Setup Guide and the SDK's tag library:

- It is staged as in a MATCH: 4 POLLEN in each FLOWER and GARDEN, 3 NECTAR in
  each upward CELL, and 4 POLLEN pre-loaded in the robot.
- The robot starts lined up with red's HIVE against the audience wall, facing
  the CELL that starts up. **P** in INIT steps through other ready-made starts
  (blue facing its HIVE, and red/blue on the audience and far walls). Clicking
  the field in INIT puts the robot at the nearest *legal* start (G304) to the
  click, on that side of the field: backed onto a wall, out of the LOADING
  ZONES, clear of the FLOWERS.
- The LOADING ZONES are labelled PARK, and the field shows LEAVE (off the wall)
  and PARK (partly in its own LOADING ZONE) live, as Section 10.5.4 scores them.
- The intake is at the back. With it running, a ball at the mouth is taken in
  (4 at most, G407); POLLEN can be pulled from a FLOWER's bottom. Anything else
  the robot touches is pushed. Reversing the intake drops the ball behind.
- A LAUNCH goes into the upward CELL the robot points at, if the flywheels are
  fast enough and the CELL is in range; otherwise it lands on the floor ahead.
  Those thresholds are guesses (`FieldSim`).
- A HIVE tips when its upward CELL holds 3 NECTAR + 3 POLLEN or 8 POLLEN (the
  setup guide's calibration table), and what was in it spills on the floor.
- The frame's two base bars and the FLOWERS stop the robot; the HIVES are above
  a robot within the 29 in. sizing volume, so it drives under them.

Not modelled: human players, scoring, FLOWER ownership, other robots, ball
height. Balls and robot are a picture, not physics.

**The position is dead reckoning.** `RobotSim` treats wheel power as wheel speed
and integrates it: no mass, no slip, and none of its speeds were measured on the
real robot. It answers "does this send the robot the way I meant", not how far
or how fast.

`gradlew :TeamCode:gamepadHarness --args="--probe"` lists the controllers it can
see. `--args="--raw"` prints what GLFW reports with no robot code in the way. A
trigger at `-1.000` there is released, not broken.

Do not use the green arrow next to `GamepadHarness.main()`. It fails with
`SourceSet with name 'unitTest' not found`: the gutter makes a Java application
configuration, and an Android module has no Java source sets. Use the Gradle
tasks above, or the checked-in run configurations in `.run/`.

## The one thing to know before writing a gamepad test

Setting a gamepad field directly does **not** produce a button edge, and in SDK
11 reading an edge **consumes** it:

```java
gamepad1.a = true;
gamepad1.aWasPressed();   // false: edges are only computed in Gamepad.copy()
```

Use `DriverStation`: hold buttons on `held1` / `held2`, then `tick()` for
`GamePadReadings`. When an OpMode is reading the pads, the runner uses
`copyIn()` instead, which leaves the edges for the OpMode; `tick()` would read
them first and the OpMode would never see the press.

## What is not covered, and why

- **`LinearOpMode`s**, which run on their own thread.
- **The camera.** `VisionPortal` cannot be constructed off the robot; `SimVision`
  stands in for the AprilTag detections.
- **Whether the Pinpoint is fitted the way the code says.** `SimPinpoint` models
  the pods and the gyro, so a wrong `setEncoderDirections`, wrong `setOffsets`,
  gyro drift or a robot moved while the gyro calibrates all misbehave here as
  they do on the robot - but only if `SimPinpoint.Mounting` describes the real
  robot. The default is a Pinpoint fitted the way `PinpointSetup` says - on
  this robot, mirrored. To try one that differs from the code, run the GUI with
  e.g. `gradlew :TeamCode:gamepadGui --args="--gui --pinpoint=x-reversed,drift=4"`
  (`x-reversed` / `y-reversed`: the forward / sideways pod counts the other way;
  `mirrored`, `not-mirrored`, `drift=<deg/s>`, `xpod=<in left>`, `ypod=<in forward>`).
- **Anything about the physical robot**: whether a power moves the robot,
  whether the launcher reaches speed, whether a ball feeds.

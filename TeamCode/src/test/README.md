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

To run a different OpMode, pass its class name:
`gradlew :TeamCode:gamepadGui --args="--gui --opmode=SomeOpMode"`.

The window shows the robot dead reckoned on a 12 ft field, the ball path
(intake, ramp, flywheels), the pad, the motor powers, and the telemetry a Driver
Station would show. A crash is shown with the line in the OpMode that threw.

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
- **The camera and the Pinpoint.** `VisionPortal` cannot be constructed off the
  robot, and there is no stand-in odometry; an OpMode that needs either is
  missing that device here.
- **Anything about the physical robot**: whether a power moves the robot,
  whether the launcher reaches speed, whether a ball feeds.

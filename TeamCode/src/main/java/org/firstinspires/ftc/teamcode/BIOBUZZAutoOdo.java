package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.devices.Drivetrain;
import org.firstinspires.ftc.teamcode.devices.DrivetrainControls;
import org.firstinspires.ftc.teamcode.devices.Intake;
import org.firstinspires.ftc.teamcode.devices.Launcher;
import org.firstinspires.ftc.teamcode.devices.PinpointSetup;
import org.firstinspires.ftc.teamcode.devices.Ramp;
import org.firstinspires.ftc.teamcode.devices.Task2;
import org.firstinspires.ftc.teamcode.utils.TelemetryLogger;

import java.util.*;
@Autonomous(name = "HiveKnightAuto-Odometry", group = "Robot")
public class BIOBUZZAutoOdo extends OpMode {
    private static final double TRANSLATION_TOLERANCE = 3;
    private static final double ROTATION_TOLERANCE = 1.0;
    private static final double driveP = 0.08; // New: 0.1
    //Original: 0.08. Controls how "careful" the robot is.
    // Unrelated to tolerance, but basically high value-fast, inaccurate and small value=slow, accurate
    private static final double rotationP = 0.08; //New: 0.04
    //Original: 0.08. Same as driveP (I think -Jasper)
    private static final double MAX_POWER = 1;
    private static final double MIN_POWER = 0.01;

    /*
     * How the Pinpoint is fitted - including that it reads as a mirror image on
     * this robot - is in devices/PinpointSetup, shared with the calibration
     * OpModes. "Cal 2: Pinpoint Check" checks it and says what to change there.
     * On 2026-11-18 the robot drove away from its target because the readings
     * were mirrored: every step it took made the error bigger.
     */

    // Heading hold while translating, so the robot does not spin as it drives.
    private static final double HEADING_HOLD_P = 0.03;
    private static final double MAX_HEADING_HOLD_POWER = 0.4;

    // If a move gets this much further from its target than it has been, the
    // odometry and the drivetrain disagree about which way is which: stop.
    private static final double RUNAWAY_INCHES = 6;

    // A still robot whose heading moves faster than this has a bad gyro calibration.
    private static final double MAX_STILL_DRIFT_DPS = 0.5;
    private static final double DRIFT_CHECK_SECONDS = 2;

    private boolean launcherStarted;

    private GoBildaPinpointDriver pinpoint;
    private Drivetrain drivetrain;
    private Intake intake;
    private Launcher launcher;
    private Ramp ramp;
    private DrivetrainControls controls;
    private TelemetryLogger log; // Telemetry that is also saved to a timestamped log file
    private boolean HiveSide;
    private boolean SideSelected;
    private boolean TaskStarted;
    private Queue<Task2> queue;
    private ElapsedTime timer;
    private ElapsedTime rotationSettleTimer = new ElapsedTime();
    private boolean rotationSettling = false;
    private static final double SETTLE_TIME = 200;
    ElapsedTime launcherTimer = new ElapsedTime();

    private double holdHeading; // the heading TRANSLATE keeps: 0 at START, then each ROTATE's target
    private double closestDistance; // how near the current TRANSLATE has got to its target
    private boolean runaway; // the auto stopped itself because a move was running away
    private final ElapsedTime driftTimer = new ElapsedTime();
    private double driftStartHeading;
    private double stillDriftDps;
    private boolean driftMeasured;

    private enum AutoMode {
        Leave,
        Park,
        Launch,
        LaunchAndPark,
        Intake,
    }

    AutoMode autoMode = null;

    @Override
    public void init() {
        log = new TelemetryLogger(telemetry, "BIOBUZZAutoOdo");
        drivetrain = new Drivetrain(hardwareMap, telemetry, false);
        controls = new DrivetrainControls();
        ramp = new Ramp(hardwareMap, telemetry);
        log.addLine("Drivetrain initialized");
        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            PinpointSetup.configure(pinpoint);
            log.addLine("Odometry initialized");
            // The gyro calibrates now and takes about 0.25 s: the robot must be still.
            pinpoint.resetPosAndIMU();
        } catch (Exception e) {
            pinpoint = null;
            log.addLine("Odometry initialization failed");
            //Not useful, as if the odometry is broken the auto and will always throw an exception.
            //Time based BIOBUZZ auto can be used instead, when it has been tested
        }
        try {
            launcher = new Launcher(hardwareMap, telemetry);
            log.addLine("Launcher initialized");
        } catch (Exception e) {
            launcher = null;
            log.addLine("Launcher initialization failed");
        }
        try {
            intake = new Intake(hardwareMap, telemetry);
        } catch (Exception e) {
            intake = null;
            log.addLine("Intake initialized");
        }
        HiveSide = true;
        SideSelected = false;
        launcherStarted = false;
        TaskStarted = false;
        queue = new LinkedList<>();
        timer = new ElapsedTime();
        log.update();

    }

    @Override
    public void init_loop() {
        if (pinpoint == null) {
            telemetry.addLine("NO PINPOINT: this auto cannot run. Check the robot configuration.");
            telemetry.update();
            return;
        }
        pinpoint.update();
        double x = PinpointSetup.forwardInches(pinpoint);   // forward
        double y = PinpointSetup.leftInches(pinpoint);      // left
        double heading = PinpointSetup.headingDegrees(pinpoint); // counterclockwise
        GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();

        // DPAD DOWN, with the robot still, recalibrates the gyro and zeroes the position.
        if (gamepad1.dpadDownWasPressed() || gamepad2.dpadDownWasPressed()) {
            pinpoint.resetPosAndIMU();
            driftMeasured = false;
            driftTimer.reset();
            driftStartHeading = heading;
        }
        // Watch the heading while the robot sits in INIT: it should not move.
        if (status != GoBildaPinpointDriver.DeviceStatus.READY) {
            driftTimer.reset();
            driftStartHeading = heading;
        } else if (driftTimer.seconds() >= DRIFT_CHECK_SECONDS) {
            stillDriftDps = AngleUnit.normalizeDegrees(heading - driftStartHeading) / driftTimer.seconds();
            driftMeasured = true;
            driftTimer.reset();
            driftStartHeading = heading;
        }

        if (!SideSelected) {
            if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
                HiveSide = true;
                SideSelected = true;
            }
            if ((gamepad1.bWasPressed() || gamepad2.bWasPressed()) && !SideSelected) {
                HiveSide = false;
                SideSelected = true;
            }
        } else if (SideSelected) {
            if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
                autoMode = AutoMode.Leave;
            } else if (gamepad1.bWasPressed() || gamepad2.bWasPressed()) {
                autoMode = AutoMode.Park;
            } else if (gamepad1.xWasPressed() || gamepad2.xWasPressed()) {
                autoMode = AutoMode.Launch;
            } else if (gamepad1.yWasPressed() || gamepad2.yWasPressed()) {
                autoMode = AutoMode.LaunchAndPark;
            } else if (gamepad1.leftBumperWasPressed() || gamepad2.leftBumperWasPressed()) {
                autoMode = AutoMode.Intake;
            } else if (gamepad1.rightBumperWasPressed() || gamepad2.rightBumperWasPressed()) {
                SideSelected = false;
                autoMode = null;
            }
        }
        if (SideSelected) {
            telemetry.addLine(HiveSide ? "Hive Side is Selected" : "Wall Side is Selected");
        } else {
            telemetry.addLine("Select Side; A=Hive, B=Wall");
        }

        if (autoMode != null) {
            telemetry.addLine(autoMode + " is Selected");
            telemetry.addLine("RB to reset");
        } else {
            telemetry.addLine("Select Auto Mode; A=Leave, B=Park, X=Launch, Y=Launch And Park, LB=Intake");
        }
        telemetry.addData("Pinpoint", status);
        if (driftMeasured && Math.abs(stillDriftDps) > MAX_STILL_DRIFT_DPS) {
            telemetry.addLine(String.format("GYRO DRIFTING %.1f deg/s. Keep the robot still and press DPAD DOWN to recalibrate.",
                    stillDriftDps));
        }
        telemetry.addData("Current X", x);
        telemetry.addData("Current Y", y);
        telemetry.addData("Current Heading", heading);
        telemetry.addLine("Bench check: push forward = X up, push left = Y up, turn left = heading up (or run Cal 2)");
        telemetry.update();
    }

    @Override
    public void start() {
        if (autoMode == null) {
            autoMode = AutoMode.Park;
        }
        //Format for tasks:
        //Start time not used for any. End time is always maximum time the task can take
        //Translate: Uses coordinates relative to field, odometry pods at (0, 0)
        //Rotate: Target heading. Not how much to rotate by, but what heading relative to field. Y parameter unused
        //Launch: How long to spin up the launcher (2 seconds should be good) and total launch time.
        //The ramp spins and causes a launch when the spin up time is completed.
        switch (autoMode) {
            case Leave:
                queue.add(new Task2(3000, Task2.Type.TRANSLATE, 0, 24)); //Time and distance settings
                break;

            case Park:
                if (HiveSide) {
                    queue.add(new Task2(2000, Task2.Type.TRANSLATE, -12, 12));
                    queue.add(new Task2(4000, Task2.Type.TRANSLATE, -36, 76));
                } else {
                    queue.add(new Task2(2000, Task2.Type.TRANSLATE, 0, 6));
                }
                break;

            case Launch:
                if (HiveSide) {
                    queue.add(new Task2(5000, Task2.Type.LAUNCH, 0.9, 2000));
                } else {
                    queue.add(new Task2(2000, Task2.Type.TRANSLATE, 0, 12));
                    queue.add(new Task2(2000, Task2.Type.TRANSLATE, 72, 12));
                    queue.add(new Task2(2000, Task2.Type.TRANSLATE, 72, 36));
                    queue.add(new Task2(3000, Task2.Type.ROTATE, -90));
                    queue.add(new Task2(2000, Task2.Type.TRANSLATE, 100, 36));
                    queue.add(new Task2(5000, Task2.Type.LAUNCH, 0.9, 2000));
                }
                break;

            case LaunchAndPark:
                if (HiveSide) {
                    queue.add(new Task2(5000, Task2.Type.LAUNCH, 0.9, 2000));
                    queue.add(new Task2(2000, Task2.Type.TRANSLATE, -12, 12));
                    queue.add(new Task2(4000, Task2.Type.TRANSLATE, -46, 96));
                } else {
                    queue.add(new Task2(2000, Task2.Type.TRANSLATE, 0, 12));
                    queue.add(new Task2(2000, Task2.Type.TRANSLATE, 72, 12));
                    queue.add(new Task2(2000, Task2.Type.TRANSLATE, 72, 36));
                    queue.add(new Task2(3000, Task2.Type.ROTATE, -90));
                    queue.add(new Task2(2000, Task2.Type.TRANSLATE, 100, 36));
                    queue.add(new Task2(5000, Task2.Type.LAUNCH, 0.9, 2000));
                    queue.add(new Task2(2000, Task2.Type.TRANSLATE, 88, 24));
                    queue.add(new Task2(4000, Task2.Type.TRANSLATE, 0, 6));
                }
                break;
            case Intake:
                if (HiveSide) {
                    queue.add(new Task2(5000, Task2.Type.LAUNCH, 0.9, 2000));
                    queue.add(new Task2(4000, Task2.Type.TRANSLATE, 0, 6));
                    queue.add(new Task2(3000, Task2.Type.ROTATE, 90));
                    queue.add(new Task2(2000, Task2.Type.TRANSLATE, -48, 6));
                    queue.add(new Task2(4000, Task2.Type.TRANSLATE, 0, 6));
                    queue.add(new Task2(3000, Task2.Type.ROTATE, 0));
                    queue.add(new Task2(5000, Task2.Type.LAUNCH, 0.9, 2000));
                    queue.add(new Task2(2000, Task2.Type.TRANSLATE, -12, 12));
                    queue.add(new Task2(4000, Task2.Type.TRANSLATE, -46, 96));
                } else {
                    queue.add(new Task2(2000, Task2.Type.TRANSLATE, 0, 12));
                    queue.add(new Task2(2000, Task2.Type.TRANSLATE, 72, 12));
                    queue.add(new Task2(2000, Task2.Type.TRANSLATE, 72, 36));
                    queue.add(new Task2(3000, Task2.Type.ROTATE, -90));
                    queue.add(new Task2(2000, Task2.Type.TRANSLATE, 100, 36));
                    queue.add(new Task2(5000, Task2.Type.LAUNCH, 0.9, 2000));
                    queue.add(new Task2(2000, Task2.Type.TRANSLATE, 94, 36));
                    queue.add(new Task2(3000, Task2.Type.ROTATE, 0));
                    queue.add(new Task2(2000, Task2.Type.TRANSLATE, 94, 0));
                    queue.add(new Task2(2000, Task2.Type.TRANSLATE, 94, 36));
                    queue.add(new Task2(3000, Task2.Type.ROTATE, -90));
                    queue.add(new Task2(5000, Task2.Type.LAUNCH, 0.9, 2000));
                    queue.add(new Task2(2000, Task2.Type.TRANSLATE, 88, 24));
                    queue.add(new Task2(4000, Task2.Type.TRANSLATE, 0, 6));
                }
        }
        // Wherever the robot is now is (0, 0), facing 0. This throws away
        // anything the Pinpoint picked up during INIT - the robot being
        // nudged into place, or gyro drift (the 2026-11-18 run started out
        // believing it faced 60 degrees).
        if (pinpoint != null) {
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        }
        holdHeading = 0;
        runaway = false;

        log.startClock();
        log.addLine("Auto mode: " + autoMode + ", " + (HiveSide ? "Hive Side" : "Wall Side"));
        if (driftMeasured && Math.abs(stillDriftDps) > MAX_STILL_DRIFT_DPS) {
            log.addLine(String.format("Started with the gyro drifting %.1f deg/s", stillDriftDps));
        }
        log.update();
        timer.reset();
    }

    @Override
    public void loop() {
        if (pinpoint == null || runaway) {
            stopEverything();
            log.addLine(pinpoint == null
                    ? "NO PINPOINT: auto not running"
                    : "STOPPED: a move ran away from its target. Check the pod directions (see the top of BIOBUZZAutoOdo).");
            log.update();
            return;
        }
        pinpoint.update();
        double x = PinpointSetup.forwardInches(pinpoint);   // forward
        double y = PinpointSetup.leftInches(pinpoint);      // left
        double heading = PinpointSetup.headingDegrees(pinpoint); // counterclockwise
        controls.translationXPower = 0;
        controls.translationYPower = 0;
        controls.rotationPower = 0;

        if (!queue.isEmpty()) {
            Task2 task2 = queue.peek();
            if (!TaskStarted) {
                timer.reset();
                TaskStarted = true;
                closestDistance = Double.MAX_VALUE;
            }
            switch (task2.type) {
                case TRANSLATE:
                    double xTarget = task2.FieldX; // Field-right, relative to the start pose
                    double yTarget = task2.FieldY; // Field-forward, relative to the start pose

                    // Pinpoint axes are X-forward and Y-left; auto targets use X-right and Y-forward.
                    double xError = yTarget - x;
                    double yError = -xTarget - y;

                    double distance = Math.hypot(xError, yError);
                    closestDistance = Math.min(closestDistance, distance);

                    if (distance > closestDistance + RUNAWAY_INCHES) {
                        // Driving towards the target is taking the robot away from it.
                        runaway = true;
                        queue.clear();
                        TaskStarted = false;
                        log.addLine(String.format("RUNAWAY: %.1f in from target after being %.1f in away",
                                distance, closestDistance));
                    } else if (distance < TRANSLATION_TOLERANCE || timer.milliseconds() > task2.end) {
                        controls.translationXPower = 0;
                        controls.translationYPower = 0;
                        controls.rotationPower = 0;

                        if (intake != null) {
                            intake.spin(0);
                        }
                        queue.remove();
                        TaskStarted = false;

                    } else {
                        double headingRad = Math.toRadians(heading); // Pinpoint heading is counterclockwise-positive
                        double sinHeading = Math.sin(headingRad);
                        double cosHeading = Math.cos(headingRad);
                        // Convert Pinpoint-frame error (forward, left) to robot controls (right, forward).
                        double robotStrafeError = xError * sinHeading - yError * cosHeading;
                        double robotForwardError = xError * cosHeading + yError * sinHeading;

                        log.addData("robotStrafeError", robotStrafeError);
                        log.addData("robotForwardError", robotForwardError);

                        controls.translationXPower = translationPower(robotStrafeError);
                        controls.translationYPower = translationPower(robotForwardError);
                        // Keep facing holdHeading while driving.
                        double headingError = AngleUnit.normalizeDegrees(holdHeading - heading);
                        controls.rotationPower = -headingHoldPower(headingError);
                        log.addData("Heading Hold Error", headingError);
                        if (intake != null) {
                            intake.spin(1);
                        }
                    }
                    log.addData("X Target", xTarget); //Unnecessary information, only used for debugging
                    log.addData("Y Target", yTarget);
                    log.addData("Heading", heading);
                    log.addData("X Error", xError);
                    log.addData("Y Error", yError);
                    log.addData("Distance Remaining", distance);
                    log.addData("X Power", controls.translationXPower);
                    log.addData("Y Power", controls.translationYPower);
                    log.addData("Task Started", TaskStarted);
                    break;

                case ROTATE:
                    double targetHeading = task2.Heading;
                    holdHeading = targetHeading; // later moves keep this heading

                    double error = targetHeading - heading;

                    while (error > 180) error -= 360;
                    while (error < -180) error += 360;

                    if (Math.abs(error) < ROTATION_TOLERANCE) {

                        controls.rotationPower = 0;

                        if (!rotationSettling) { //Possibly not necessary
                            rotationSettleTimer.reset();
                            rotationSettling = true;
                        }
                        if (rotationSettleTimer.milliseconds() >= SETTLE_TIME) {
                            queue.remove();
                            TaskStarted = false;
                            rotationSettling = false;
                        }
                    } else {
                        rotationSettling = false;
                        // Positive drivetrain rotation is clockwise; Pinpoint heading is counterclockwise-positive.
                        controls.rotationPower = -rotationPower(error);
                    }

                    log.addData("Heading Error", error);
                    log.addData("Settling", rotationSettling);

                    break;

                case LAUNCH:
                    if (intake != null && launcher != null) {

                        if (!launcherStarted) {
                            launcherTimer.reset();
                            launcherStarted = true;
                        }

                        if (launcherTimer.milliseconds() < task2.FieldY) {
                            launcher.spin(task2.FieldX);
                        } else if (launcherTimer.milliseconds() < task2.end) {
                            launcher.spin(task2.LauncherPower);
                            ramp.spin(1);
                        } else {
                            launcher.spin(0);
                            ramp.spin(0);

                            queue.remove();
                            launcherStarted = false;
                            TaskStarted = false;
                        }

                    } else {
                        queue.remove();
                        launcherStarted = false;
                        TaskStarted = false;
                    }
                    break;

                default:
                    controls.translationXPower = 0;
                    controls.translationYPower = 0;
                    controls.rotationPower = 0;
                    TaskStarted = false;
                    launcherStarted = false;
                    queue.remove();
                    break;
            }
        } else {
            if (intake != null) intake.spin(0);
            if (launcher != null) launcher.spin(0);
            if (ramp != null) ramp.spin(0);

            if (drivetrain != null) {
                controls.translationXPower = 0;
                controls.translationYPower = 0;
                controls.rotationPower = 0;
            }
        }

        if (drivetrain != null) drivetrain.run(controls);
        log.addData("Current X", x);
        log.addData("Current Y", y);
        log.addData("Current Heading", heading);
        log.addData("Remaining Tasks", queue.size());
        log.update();
    }

    @Override
    public void stop() {
        if (log != null) log.close();
    }

    private double translationPower(double error) {
        double power = error * driveP;

        power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));

        if (Math.abs(power) < MIN_POWER && error != 0) {
            power = Math.signum(power) * MIN_POWER;
        }
        return power;
    }

    /** Gentle turn to hold a heading while translating; positive is counterclockwise. */
    private double headingHoldPower(double error) {
        if (Math.abs(error) < ROTATION_TOLERANCE) {
            return 0;
        }
        double power = error * HEADING_HOLD_P;
        return Math.max(-MAX_HEADING_HOLD_POWER, Math.min(MAX_HEADING_HOLD_POWER, power));
    }

    private void stopEverything() {
        controls.translationXPower = 0;
        controls.translationYPower = 0;
        controls.rotationPower = 0;
        if (drivetrain != null) drivetrain.run(controls);
        if (intake != null) intake.spin(0);
        if (launcher != null) launcher.spin(0);
        if (ramp != null) ramp.spin(0);
    }

    private double rotationPower(double error) {

        if (Math.abs(error) < ROTATION_TOLERANCE) {
            return 0;
        }
        double power = error * rotationP;

        power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));

        if (Math.abs(power) < MIN_POWER) {
            power = Math.signum(power) * MIN_POWER;
        }

        return power;
    }

}
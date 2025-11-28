/*
 * Copyright (c) 2025 Base 10 Assets, LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of NAME nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static org.firstinspires.ftc.teamcode.utils.Utils.applyAction;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.devices.GoBildaPinpointDriver;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Locale;

/*
 * This file includes an autonomous file for the goBILDA® StarterBot for the
 * 2025-2026 FIRST® Tech Challenge season DECODE™. It leverages a differential/Skid-Steer
 * system for robot mobility, one high-speed motor driving two "launcher wheels," and two servos
 * which feed that launcher.
 *
 * This robot starts up against the goal and launches all three projectiles before driving away
 * off the starting line.
 *
 * This program leverages a "state machine" - an Enum which captures the state of the robot
 * at any time. As it moves through the autonomous period and completes different functions,
 * it will move forward in the enum. This allows us to run the autonomous period inside of our
 * main robot "loop," continuously checking for conditions that allow us to move to the next step.
 */

@Autonomous(name = "StarterBotAuto", group = "StarterBot")
// @Disabled
public class StarterBotAuto extends OpMode {

    final double FEED_TIME = 0.50; // The feeder servos run this long when a shot is requested.

    /*
     * When we control our launcher motor, we are using encoders. These allow the
     * control system
     * to read the current speed of the motor and apply more or less power to keep
     * it at a constant
     * velocity. Here we are setting the target and minimum velocity that the
     * launcher should run
     * at. The minimum velocity is a threshold for determining when to fire.
     */
    final double LAUNCHER_TARGET_VELOCITY_1 = 1687.5;
    final double LAUNCHER_TARGET_VELOCITY_2 = 1900;
    final double LAUNCHER_TARGET_VELOCITY_3 = 2250;
    /*
     * The number of seconds that we wait between each of our 3 shots from the
     * launcher. This
     * can be much shorter, but the longer break is reasonable since it maximizes
     * the likelihood
     * that each shot will score.
     */
    double TIME_BETWEEN_SHOTS = 3;

    /*
     * Here we capture a few variables used in driving the robot. DRIVE_SPEED and
     * ROTATE_SPEED
     * are from 0-1, with 1 being full speed. Encoder ticks per revolution is
     * specific to the motor
     * ratio that we use in the kit; if you're using a different motor, this value
     * can be found on
     * the product page for the motor you're using.
     * Track width is the distance between the center of the drive wheels on either
     * side of the
     * robot. Track width is used to determine the amount of linear distance each
     * wheel needs to
     * travel to create a specified rotation of the robot.
     */
    final double DRIVE_SPEED = 0.5;
    final double ROTATE_SPEED = 0.2;
    final double WHEEL_DIAMETER_MM = 140; // default: 96
    final double ENCODER_TICKS_PER_REV = 806.55; // default: 537.7;
    final double TICKS_PER_MM = (ENCODER_TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI));
    final double TRACK_WIDTH_MM = 419; // default: 404

    int shotsToFire = 3; // The number of shots to fire in this auto.

    double robotRotationAngle = 45;

    double feederReloadAngle = 0.5;
    double feederFireAngle = 0.25;

    final double TURRET_TICKS_PER_DEGREE = 5272.0 / 360;

    final int RESOLUTION_WIDTH = 640;
    final int RESOLUTION_HEIGHT = 480;

    final double TAG_X_OFFSET = 3;

    final double AIM_TIME_SECONDS = 0.50;
    final double AIM_MAX_TIME_SECONDS = 3;

    double opTimeLimit = 10; // seconds

    protected double launcherSpeed = LAUNCHER_TARGET_VELOCITY_1;
    /*
     * Here we create three timers which we use in different parts of our code. Each
     * of these is an
     * "object," so even though they are all an instance of ElapsedTime(), they
     * count independently
     * from each other.
     */
    private final ElapsedTime shotTimer = new ElapsedTime();
    protected final ElapsedTime feederTimer = new ElapsedTime();
    private final ElapsedTime driveTimer = new ElapsedTime();
    private final ElapsedTime selectButtonTimer = new ElapsedTime();
    protected final ElapsedTime aimTimer = new ElapsedTime();
    protected final ElapsedTime aimMaxTimer = new ElapsedTime();
    private final ElapsedTime opMaxTimer = new ElapsedTime();

    // Declare OpMode members.
    protected DcMotor leftFrontDrive = null;
    protected DcMotor rightFrontDrive = null;
    protected DcMotor leftBackDrive = null;
    protected DcMotor rightBackDrive = null;
    protected DcMotorEx[] launchers = null;
    protected Servo feeder = null;
    protected DcMotorEx frontIntakeWheel = null;
    protected DcMotorEx turret = null;

    private DigitalChannel turretHomeSwitch = null;

    protected GoBildaPinpointDriver odo = null;

    private AprilTagProcessor aprilTag = null;
    protected VisionPortal portal = null;
    protected AprilTagDetection targetTag = null;

    /*
     * TECH TIP: State Machines
     * We use "state machines" in a few different ways in this auto. The first step
     * of a state
     * machine is creating an enum that captures the different "states" that our
     * code can be in.
     * The core advantage of a state machine is that it allows us to continue to
     * loop through code,
     * and only run the bits of code we need to at different times. This state
     * machine is called the
     * "LaunchState." It reflects the current condition of the shooter motor when we
     * request a shot.
     * It starts at IDLE. When a shot is requested from the user, it'll move into
     * PREPARE then LAUNCH.
     * We can use higher level code to cycle through these states, but this allows
     * us to write
     * functions and autonomous routines in a way that avoids loops within loops,
     * and "waits."
     */
    protected enum LaunchState {
        IDLE,
        AIM,
        PREPARE,
        LAUNCH,
    }

    /*
     * Here we create the instance of LaunchState that we use in code. This creates
     * a unique object
     * which can store the current condition of the shooter. In other applications,
     * you may have
     * multiple copies of the same enum which have different names. Here we just
     * have one.
     */
    protected LaunchState launchState;

    /*
     * Here is our auto state machine enum. This captures each action we'd like to
     * do in auto.
     */
    private enum AutonomousState {
        DRIVING_TO_LINE,
        ROTATE_TO_LINE,
        DRIVE_TO_GOAL,
        LAUNCH,
        WAIT_FOR_LAUNCH,
        DRIVING_AWAY_FROM_GOAL,
        ROTATING,
        DRIVING_OFF_LINE,
        COMPLETE,
    }

    private AutonomousState autonomousState = AutonomousState.DRIVING_TO_LINE;

    /*
     * Here we create an enum not to create a state machine, but to capture which
     * alliance we are on.
     */
    protected enum Alliance {
        RED,
        BLUE
    }

    /*
     * When we create the instance of our enum we can also assign a default state.
     */
    protected Alliance alliance = Alliance.BLUE;

    private enum StartPosition {
        TBD,
        NEAR,
        FAR
    }

    private StartPosition startPosition = StartPosition.FAR;

    protected int targetTagId = 20;

    protected boolean drivetrainOnly = true;

    protected boolean reverseRotate = false;

    /*
     * This code runs ONCE when the driver hits INIT.
     */
    @Override
    public void init() {
        /*
         * Here we set the first step of our autonomous state machine by setting
         * autoStep = AutoStep.LAUNCH.
         * Later in our code, we will progress through the state machine by moving to
         * other enum members.
         * We do the same for our launcher state machine, setting it to IDLE before we
         * use it later.
         */
        launchState = LaunchState.IDLE;

        /*
         * Initialize the hardware variables. Note that the strings used here as
         * parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the driver's station).
         */
        leftFrontDrive = hardwareMap.tryGet(DcMotor.class, "left_front_drive");
        if (leftFrontDrive != null) {
            leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
            rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
            leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
            rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        } else {
            leftFrontDrive = hardwareMap.get(DcMotor.class, "rbdrive");
            rightFrontDrive = hardwareMap.get(DcMotor.class, "lbdrive");
            leftBackDrive = hardwareMap.get(DcMotor.class, "rfdrive");
            rightBackDrive = hardwareMap.get(DcMotor.class, "lfdrive");
            reverseRotate = true;
            opTimeLimit = 1;
        }

        launchers = new DcMotorEx[] { hardwareMap.tryGet(DcMotorEx.class, "leftLauncher"),
                hardwareMap.tryGet(DcMotorEx.class, "rightLauncher") };
        feeder = hardwareMap.tryGet(Servo.class, "feeder");
        frontIntakeWheel = hardwareMap.tryGet(DcMotorEx.class, "intake");
        turret = hardwareMap.tryGet(DcMotorEx.class, "turret");
        turretHomeSwitch = hardwareMap.tryGet(DigitalChannel.class, "turretHomeSwitch");

        odo = hardwareMap.tryGet(GoBildaPinpointDriver.class, "odo");

        WebcamName webcam = hardwareMap.tryGet(WebcamName.class, "Webcam 1");

        /*
         * To drive forward, most robots need the motor on one side to be reversed,
         * because the axles point in opposite directions. Pushing the left stick
         * forward
         * MUST make the robot go forward. So, adjust these two lines based on your
         * first test drive.
         * Note: The settings here assume direct drive on left and right wheels. Gear
         * Reduction or 90° drives may require direction flips
         */
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        /*
         * Here we reset the encoders on our drive motors before we start moving.
         */
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode." This causes the
         * motor to
         * slow down much faster when it is coasting. This creates a much more
         * controllable
         * drivetrain, as the robot stops much quicker.
         */
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        if (launchers[0] != null) {
            drivetrainOnly = false;
            applyAction(launchers, (motor) -> motor.setZeroPowerBehavior(BRAKE));

            /*
             * Here we set our launcher to the RUN_USING_ENCODER runmode.
             * If you notice that you have no control over the velocity of the motor, and it
             * just jumps
             * right to a number much higher than your set point, make sure that your
             * encoders are plugged
             * into the port right beside the motor itself.
             */
            applyAction(launchers, (motor) -> motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER));

            /*
             * Here we set the aforementioned PID coefficients. You shouldn't have to do
             * this for any
             * other motors on this robot.
             */
            applyAction(launchers, (motor) -> motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(300, 0, 0, 10)));
            launchers[0].setDirection(DcMotor.Direction.REVERSE);
            launchers[1].setDirection(DcMotor.Direction.FORWARD);

            turret.setZeroPowerBehavior(BRAKE);
            turretHomeSwitch.setMode(DigitalChannel.Mode.INPUT);

            /*
             * Much like our drivetrain motors, we set the left feeder servo to reverse so
             * that they
             * both work to feed the ball into the robot.
             */
            feeder.resetDeviceConfigurationForOpMode();
            feeder.setPosition(feederReloadAngle);
            frontIntakeWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontIntakeWheel.setZeroPowerBehavior(BRAKE);
            telemetry.addLine("init position" + feeder.getPosition());
            telemetry.addLine("range=" + feederReloadAngle + ", " + feederFireAngle);
        }

        if (odo != null) {
            odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);
            odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                    GoBildaPinpointDriver.EncoderDirection.FORWARD);
            odo.resetPosAndIMU();
        }

        if (webcam != null) {
            // Initialize AprilTag processor
            long acquTime = System.nanoTime();
            aprilTag = new AprilTagProcessor.Builder()
                    .setDrawAxes(true)
                    .setDrawCubeProjection(true)
                    .setDrawTagOutline(true)
                    .setCameraPose(
                            new Position(DistanceUnit.INCH, 9, 4, 17, acquTime),
                            new YawPitchRollAngles(AngleUnit.DEGREES, 0, 15, 0, acquTime))
                    // Optional: tune camera intrinsics here if you have calibration data
                    // .setLensIntrinsics(fx, fy, cx, cy)
                    .build();

            portal = new VisionPortal.Builder()
                    .setCamera(webcam)
                    .setCameraResolution(new Size(RESOLUTION_WIDTH, RESOLUTION_HEIGHT))
                    .addProcessor(aprilTag)
                    .build();
        }

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * This code runs REPEATEDLY after the driver hits INIT, but before they hit
     * START.
     */
    @Override
    public void init_loop() {
        /*
         * Here we allow the driver to select which alliance we are on using the
         * gamepad.
         */
        if (gamepad1.back && selectButtonTimer.seconds() > 0.5) {
            selectButtonTimer.reset();
            if (alliance == Alliance.BLUE && startPosition == StartPosition.FAR) {
                startPosition = StartPosition.NEAR;
            } else if (alliance == Alliance.BLUE && startPosition == StartPosition.NEAR) {
                alliance = Alliance.RED;
                startPosition = StartPosition.FAR;
            } else if (alliance == Alliance.RED && startPosition == StartPosition.FAR) {
                startPosition = StartPosition.NEAR;
            } else if (alliance == Alliance.RED && startPosition == StartPosition.NEAR) {
                alliance = Alliance.BLUE;
                startPosition = StartPosition.FAR;
            }
        }
        if (gamepad1.y) {
            drivetrainOnly = !drivetrainOnly;
            if (launchers[0] == null) {
                drivetrainOnly = true;
            }
        }

        if (alliance == Alliance.BLUE) {
            targetTagId = 20;
        } else {
            targetTagId = 24;
        }

        if (startPosition == StartPosition.FAR) {
            autonomousState = AutonomousState.DRIVING_TO_LINE;
        } else {
            autonomousState = AutonomousState.DRIVING_AWAY_FROM_GOAL;
        }

        targetTag = locateTarget(targetTagId);

        telemetry.addData("Press BACK", "select ALLIANCE and POSITION");
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Position", startPosition);
        telemetry.addLine();
        telemetry.addData("Press Y", "toggle launcher");
        telemetry.addData("Launcher enabled", !drivetrainOnly);
        telemetry.addLine();
        if (targetTag != null) {
            telemetry.addData("Tag ID", targetTag.metadata.id);
            telemetry.addData("Tag X", targetTag.ftcPose.x);
            telemetry.addData("Tag Y", targetTag.ftcPose.y);
        }
        if (odo != null) {
            telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
            telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.MM));
            telemetry.addData("Heading Scalar", odo.getYawScalar());
        }
    }

    /*
     * This code runs ONCE when the driver hits START.
     */
    @Override
    public void start() {
        opMaxTimer.reset();
    }

    /*
     * This code runs REPEATEDLY after the driver hits START but before they hit
     * STOP.
     */
    @Override
    public void loop() {
        /*
         * TECH TIP: Switch Statements
         * switch statements are an excellent way to take advantage of an enum. They
         * work very
         * similarly to a series of "if" statements, but allow for cleaner and more
         * readable code.
         * We switch between each enum member and write the code that should run when
         * our enum
         * reflects that state. We end each case with "break" to skip out of checking
         * the rest
         * of the members of the enum for a match, since if we find the "break" line in
         * one case,
         * we know our enum isn't reflecting a different state.
         */
        String data = "";
        if (odo != null) {
            odo.update();
            Pose2D pos = odo.getPosition();
            data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM),
                    pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
        }

        targetTag = locateTarget(targetTagId);

        double speed = startPosition == StartPosition.NEAR ? LAUNCHER_TARGET_VELOCITY_1
                : LAUNCHER_TARGET_VELOCITY_3;

        switch (autonomousState) {
            /*
             * Since the first state of our auto is LAUNCH, this is the first "case" we
             * encounter.
             * This case is very simple. We call our .launch() function with "true" in the
             * parameter.
             * This "true" value informs our launch function that we'd like to start the
             * process of
             * firing a shot. We will call this function with a "false" in the next case.
             * This
             * "false" condition means that we are continuing to call the function every
             * loop,
             * allowing it to cycle through and continue the process of launching the first
             * ball.
             */
            case DRIVING_TO_LINE:
                if (drive(DRIVE_SPEED, 50, DistanceUnit.INCH, 1, 0.5)) {
                    leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    autonomousState = AutonomousState.ROTATE_TO_LINE;
                    opMaxTimer.reset();
                }
                break;

            case ROTATE_TO_LINE:
                if (alliance == Alliance.BLUE) {
                    robotRotationAngle = -45;
                } else if (alliance == Alliance.RED) {
                    robotRotationAngle = 45;
                }

                if (rotate(ROTATE_SPEED, robotRotationAngle, AngleUnit.DEGREES, 1, 0.5)) {
                    leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    autonomousState = AutonomousState.LAUNCH;
                    opMaxTimer.reset();
                }
                break;

            case LAUNCH:
                leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                launch(true, speed);
                autonomousState = AutonomousState.WAIT_FOR_LAUNCH;
                break;

            case WAIT_FOR_LAUNCH:
                /*
                 * A technique we leverage frequently in this code are functions which return a
                 * boolean. We are using this function in two ways. This function actually moves
                 * the
                 * motors and servos in a way that launches the ball, but it also "talks back"
                 * to
                 * our main loop by returning either "true" or "false". We've written it so that
                 * after the shot we requested has been fired, the function will return "true"
                 * for
                 * one cycle. Once the launch function returns "true", we proceed in the code,
                 * removing
                 * one from the shotsToFire variable. If shots remain, we move back to the
                 * LAUNCH
                 * state on our state machine. Otherwise, we reset the encoders on our drive
                 * motors
                 * and move onto the next state.
                 */
                mecanumDrive(0, 0, 0);
                if (launch(false, speed)) {
                    shotsToFire -= 1;
                    if (shotsToFire > 0) {
                        autonomousState = AutonomousState.LAUNCH;
                    } else {
                        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        if (!drivetrainOnly) {
                            applyAction(launchers, (launcher) -> launcher.setVelocity(0));
                        }
                        autonomousState = AutonomousState.COMPLETE;
                    }
                }
                break;

            case DRIVING_AWAY_FROM_GOAL:
                /*
                 * This is another function that returns a boolean. This time we return "true"
                 * if
                 * the robot has been within a tolerance of the target position for
                 * "holdSeconds."
                 * Once the function returns "true" we reset the encoders again and move on.
                 */
                if (drive(DRIVE_SPEED, -30, DistanceUnit.INCH, 1, 0.5)) {
                    leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    autonomousState = AutonomousState.LAUNCH;
                }
                break;
        }

        /*
         * Here is our telemetry that keeps us informed of what is going on in the
         * robot. Since this
         * part of the code exists outside of our switch statement, it will run once
         * every loop.
         * No matter what state our robot is in. This is the huge advantage of using
         * state machines.
         * We can have code inside of our state machine that runs only when necessary,
         * and code
         * after the last "case" that runs every loop. This means we can avoid a lot of
         * "copy-and-paste" that non-state machine autonomous routines fall into.
         */
        telemetry.addData("AutoState", autonomousState);
        telemetry.addData("LauncherState", launchState);
        telemetry.addData("Motor Current Positions", "left (%d), right (%d)",
                leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition());
        telemetry.addData("Motor Target Positions", "left (%d), right (%d)",
                leftFrontDrive.getTargetPosition(), rightFrontDrive.getTargetPosition());
        telemetry.addData("Position", data);
        if (targetTag != null) {
            telemetry.addData("Tag ID", targetTag.metadata.id);
            telemetry.addData("Tag X", targetTag.ftcPose.x);
            telemetry.addData("Tag Y", targetTag.ftcPose.y);
        }
        telemetry.update();
    }

    /*
     * This code runs ONCE after the driver hits STOP.
     */
    @Override
    public void stop() {
    }

    /**
     * Launches one ball, when a shot is requested spins up the motor and once it is
     * above a minimum
     * velocity, runs the feeder servos for the right amount of time to feed the
     * next ball.
     *
     * @param shotRequested "true" if the user would like to fire a new shot, and
     *                      "false" if a shot
     *                      has already been requested and we need to continue to
     *                      move through the
     *                      state machine and launch the ball.
     * @return "true" for one cycle after a ball has been successfully launched,
     *         "false" otherwise.
     */
    boolean launch(boolean shotRequested, double speed) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launcherSpeed = speed;
                    launchState = LaunchState.AIM;
                    aimTimer.reset();
                    aimMaxTimer.reset();
                }
                break;

            case AIM:
                double x_pos = aim(1);

                if (Math.abs(x_pos) >= 1) {
                    aimTimer.reset();
                }

                if (aimTimer.seconds() > AIM_TIME_SECONDS || aimMaxTimer.seconds() > AIM_MAX_TIME_SECONDS) {
                    launchState = LaunchState.PREPARE;
                    shotTimer.reset();
                    mecanumDrive(0, 0, 0);
                }

                break;

            case PREPARE:
                if (!drivetrainOnly) {
                    applyAction(launchers, (launcher) -> launcher.setVelocity(launcherSpeed));

                    if (launchers[0].getVelocity() > launcherSpeed - 25
                            && launchers[1].getVelocity() > launcherSpeed - 25) {
                        launchState = LaunchState.LAUNCH;
                        feederTimer.reset();
                    }
                } else {
                    launchState = LaunchState.LAUNCH;
                    feederTimer.reset();
                }
                break;

            case LAUNCH:
                if (!drivetrainOnly) {
                    feeder.setPosition(feederFireAngle);

                    if (feederTimer.seconds() > FEED_TIME) {
                        applyAction(launchers, (launcher) -> launcher.setVelocity(0));
                        feeder.setPosition(feederReloadAngle);

                        if (shotTimer.seconds() > TIME_BETWEEN_SHOTS) {
                            launchState = LaunchState.IDLE;
                            frontIntakeWheel.setPower(0);
                            return true;
                        } else {
                            frontIntakeWheel.setPower(-1);
                        }
                    }
                } else {
                    launchState = LaunchState.IDLE;
                    return true;
                }
        }
        return false;
    }

    /**
     * @param speed        From 0-1
     * @param distance     In specified unit
     * @param distanceUnit the unit of measurement for distance
     * @param holdSeconds  the number of seconds to wait at position before
     *                     returning true.
     * @return "true" if the motors are within tolerance of the target position for
     *         more than
     *         holdSeconds. "false" otherwise.
     */
    boolean drive(double speed, double distance, DistanceUnit distanceUnit, double holdSeconds, double maxSeconds) {
        final double TOLERANCE_MM = 10;
        /*
         * In this function we use a DistanceUnits. This is a class that the FTC SDK
         * implements
         * which allows us to accept different input units depending on the user's
         * preference.
         * To use these, put both a double and a DistanceUnit as parameters in a
         * function and then
         * call distanceUnit.toMm(distance). This will return the number of mm that are
         * equivalent
         * to whatever distance in the unit specified. We are working in mm for this, so
         * that's the
         * unit we request from distanceUnit. But if we want to use inches in our
         * function, we could
         * use distanceUnit.toInches() instead!
         */
        double targetPosition = (distanceUnit.toMm(distance) * TICKS_PER_MM);

        leftFrontDrive.setTargetPosition((int) targetPosition);
        rightFrontDrive.setTargetPosition((int) targetPosition);
        leftBackDrive.setTargetPosition((int) targetPosition);
        rightBackDrive.setTargetPosition((int) targetPosition);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightBackDrive.setPower(speed);

        /*
         * Here we check if we are within tolerance of our target position or not. We
         * calculate the
         * absolute error (distance from our setpoint regardless of if it is positive or
         * negative)
         * and compare that to our tolerance. If we have not reached our target yet,
         * then we reset
         * the driveTimer. Only after we reach the target can the timer count higher
         * than our
         * holdSeconds variable.
         */
        if (Math.abs(targetPosition - leftFrontDrive.getCurrentPosition()) > (TOLERANCE_MM * TICKS_PER_MM)) {
            driveTimer.reset();
        }

        if (opMaxTimer.seconds() > maxSeconds || opMaxTimer.seconds() > opTimeLimit) {
            return true;
        }

        return (driveTimer.seconds() > holdSeconds);
    }

    /**
     * @param speed       From 0-1
     * @param angle       the amount that the robot should rotate
     * @param angleUnit   the unit that angle is in
     * @param holdSeconds the number of seconds to wait at position before returning
     *                    true.
     * @return True if the motors are within tolerance of the target position for
     *         more than
     *         holdSeconds. False otherwise.
     */
    boolean rotate(double speed, double angle, AngleUnit angleUnit, double holdSeconds, double maxSeconds) {
        final double TOLERANCE_MM = 10;

        /*
         * Here we establish the number of mm that our drive wheels need to cover to
         * create the
         * requested angle. We use radians here because it makes the math much easier.
         * Our robot will have rotated one radian when the wheels of the robot have
         * driven
         * 1/2 of the track width of our robot in a circle. This is also the radius of
         * the circle
         * that the robot tracks when it is rotating. So, to find the number of mm that
         * our wheels
         * need to travel, we just need to multiply the requested angle in radians by
         * the radius
         * of our turning circle.
         */
        double targetMm = angleUnit.toRadians(angle) * (TRACK_WIDTH_MM / 2);

        /*
         * We need to set the left motor to the inverse of the target so that we rotate
         * instead
         * of driving straight.
         */
        double leftTargetPosition = -(targetMm * TICKS_PER_MM);
        double rightTargetPosition = targetMm * TICKS_PER_MM;

        leftFrontDrive.setTargetPosition((int) leftTargetPosition);
        rightFrontDrive.setTargetPosition((int) rightTargetPosition);
        leftBackDrive.setTargetPosition((int) leftTargetPosition);
        rightBackDrive.setTargetPosition((int) rightTargetPosition);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightBackDrive.setPower(speed);

        if ((Math.abs(leftTargetPosition - leftFrontDrive.getCurrentPosition())) > (TOLERANCE_MM * TICKS_PER_MM)) {
            driveTimer.reset();
        }

        if (opMaxTimer.seconds() > maxSeconds || opMaxTimer.seconds() > opTimeLimit) {
            return true;
        }

        return (driveTimer.seconds() > holdSeconds);
    }

    protected AprilTagDetection locateTarget(int targetTagId) {
        AprilTagDetection targetTag = null;

        if (aprilTag != null) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null && detection.metadata.id == targetTagId) {
                    targetTag = detection;
                    break;
                }
            }
        }

        return targetTag;
    }

    void mecanumDrive(double forward, double strafe, double rotate) {

        /*
         * the denominator is the largest motor power (absolute value) or 1
         * This ensures all the powers maintain the same ratio,
         * but only if at least one is out of the range [-1, 1]
         */
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        double leftFrontPower = (forward + strafe + rotate) / denominator;
        double rightFrontPower = (forward - strafe - rotate) / denominator;
        double leftBackPower = (forward - strafe + rotate) / denominator;
        double rightBackPower = (forward + strafe - rotate) / denominator;

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    protected double aim(double tolerance) {
        if (targetTag == null) {
            return 0;
        }
        double x_pos = targetTag.ftcPose.x + TAG_X_OFFSET;
        if (Math.abs(x_pos) < tolerance) {
            return x_pos;
        }
        double rotate_power = Math.max(0.15, Math.abs(x_pos) / 16) * (reverseRotate ? -1 : 1)
                * (x_pos > 0 ? 1 : -1);
        mecanumDrive(0, 0, rotate_power);
        return x_pos;
    }
}

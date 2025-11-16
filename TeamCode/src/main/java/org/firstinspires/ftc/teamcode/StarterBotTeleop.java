
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.data.AimingParameters;
import org.firstinspires.ftc.teamcode.data.Target;
import org.firstinspires.ftc.teamcode.devices.*;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Locale;

@TeleOp(name = "StarterBotTeleop-2025Decode", group = "StarterBot")
//@Disabled
public class StarterBotTeleop extends OpMode {
    /*
     * When we control our launcher motor, we are using encoders. These allow the control system
     * to read the current speed of the motor and apply more or less power to keep it at a constant
     * velocity. Here we are setting the target, and minimum velocity that the launcher should run
     * at. The minimum velocity is a threshold for determining when to fire.
     */

    public enum StartPosition{
        TBD,
        NEAR,
        FAR
    }
    private Launcher launcher = null;
    private Drivetrain drivetrain = null;
    private Intake intake = null;
    private boolean autoMode = false;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private int targetTagId = 20; // default to Team BLUE
    private StartPosition startPosition = StartPosition.TBD;

    /*
     * TECH TIP: State Machines
     * We use a "state machine" to control our launcher motor and feeder servos in this program.
     * The first step of a state machine is creating an enum that captures the different "states"
     * that our code can be in.
     * The core advantage of a state machine is that it allows us to continue to loop through all
     * of our code while only running specific code when it's necessary. We can continuously check
     * what "State" our machine is in, run the associated code, and when we are done with that step
     * move on to the next state.
     * This enum is called the "LaunchState". It reflects the current condition of the shooter
     * motor and we move through the enum when the user asks our code to fire a shot.
     * It starts at idle, when the user requests a launch, we enter SPIN_UP where we get the
     * motor up to speed, once it meets a minimum speed then it starts and then ends the launch process.
     * We can use higher level code to cycle through these states. But this allows us to write
     * functions and autonomous routines in a way that avoids loops within loops, and "waits".
     */

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
         drivetrain = new Drivetrain(hardwareMap, telemetry, false);
         launcher = new Launcher(hardwareMap, telemetry);
         intake = new Intake(hardwareMap, telemetry);
         telemetry.addData("Status", "Motors Initialized");

        // Initialize AprilTag processor
        long acquTime = System.nanoTime();
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setCameraPose(
                        new Position(DistanceUnit.INCH, 9, 4, 17, acquTime),
                        new YawPitchRollAngles( AngleUnit.DEGREES,0, 15, 0, acquTime))
                // Optional: tune camera intrinsics here if you have calibration data
                // .setLensIntrinsics(fx, fy, cx, cy)
                .build();

        // Create vision portal using the built-in webcam
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        telemetry.addLine("Camera initialized. Waiting for start...");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        if (gamepad1.xWasPressed() || gamepad2.xWasPressed()) {
            targetTagId = 20;
            telemetry.addLine("Team BLUE is selected");
        }

        if (gamepad1.bWasPressed() || gamepad2.bWasPressed()){
            targetTagId = 24;
            telemetry.addLine("Team RED is selected.");
        }
        if (gamepad1.yWasPressed() || gamepad2.yWasPressed()) {
            startPosition = StartPosition.FAR;
            telemetry.addLine("Starting position FAR.");
           // launcher.setTurretAngle(60, 5);
        }

        if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
            startPosition = StartPosition.NEAR;
            telemetry.addLine("Starting position NEAR.");
         //   launcher.setTurretAngle(30, 5);
        }
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        telemetry.addData("Target ID", targetTagId);
        telemetry.addData("Starting from", startPosition);
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        /*
         * Here we call a function called arcadeDrive. The arcadeDrive function takes the input from
         * the joysticks, and applies power to the left and right drive motor to move the robot
         * as requested by the driver. "arcade" refers to the control style we're using here.
         * Much like a classic arcade game, when you move the left joystick forward both motors
         * work to drive the robot forward, and when you move the right joystick left and right
         * both motors work to rotate the robot. Combinations of these inputs can be used to create
         * more complex maneuvers.
         */
        // Sensing
        // This is the global readings.
        GamePadReadings gamepadReading = new GamePadReadings(){{
            bButton = gamepad1.b || gamepad2.b;
            aButton = gamepad1.a || gamepad2.a;
            xButton = gamepad1.x || gamepad2.x;
            yButton = gamepad1.y || gamepad2.y;
            leftBumper = gamepad1.left_bumper || gamepad2.left_bumper;
            rightBumper = gamepad1.right_bumper || gamepad2.right_bumper;

            // Since stick reading ranges from -1 to 1, we use the reading with mas absolute value.
            leftStickX =  Math.abs(gamepad1.left_stick_x) >= Math.abs(gamepad2.left_stick_x) ?
                gamepad1.left_stick_x : gamepad2.left_stick_x;
            leftStickY = Math.abs(gamepad1.left_stick_y) >= Math.abs(gamepad2.left_stick_y) ?
                    gamepad1.left_stick_y : gamepad2.left_stick_y;
            rightStickX = Math.abs(gamepad1.right_stick_x) >= Math.abs(gamepad2.right_stick_x) ?
                    gamepad1.right_stick_x : gamepad2.right_stick_x;
            rightStickY =  Math.abs(gamepad1.right_stick_y) >= Math.abs(gamepad2.right_stick_y) ?
                    gamepad1.right_stick_y : gamepad2.right_stick_y;


            leftTrigger = Math.max(gamepad1.left_trigger, gamepad2.left_trigger);
            rightTrigger = Math.max(gamepad1.right_trigger, gamepad2.right_trigger);

            dPadUp = gamepad1.dpad_up || gamepad2.dpad_up;
            dPadDown = gamepad1.dpad_down || gamepad2.dpad_down;
            dPadLeft = gamepad1.dpad_left || gamepad2.dpad_left;
            dPadRight = gamepad1.dpad_right || gamepad2.dpad_right;
            dPad = dPadDown && dPadUp && dPadLeft && dPadRight;

            aWasReleased = gamepad1.aWasReleased() || gamepad2.aWasReleased();
            aWasPressed = gamepad1.aWasPressed() || gamepad2.aWasPressed();

            bWasReleased = gamepad1.bWasReleased() || gamepad2.bWasReleased();
            bWasPressed = gamepad1.bWasPressed() || gamepad2.bWasPressed();

            xWasReleased = gamepad1.xWasReleased() || gamepad2.xWasReleased();
            xWasPressed = gamepad1.xWasPressed() || gamepad2.xWasPressed();

            yWasReleased = gamepad1.yWasReleased() || gamepad2.yWasReleased();
            yWasPressed = gamepad1.yWasPressed() || gamepad2.yWasPressed();
        }};

        AprilTagDetection targetTag = null;
        Target target = null;

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && detection.metadata.id == 20) {
                if (targetTagId == detection.metadata.id)
                    targetTag = detection;
            }

            if (detection.metadata != null && detection.metadata.id == 24) {
                if (targetTagId == detection.metadata.id)
                    targetTag = detection;
            }
        }

        /* We can also use the tag position to determine the robot position for navigation.*/

        if (targetTag != null){
            double a = Math.toDegrees(Math.atan2(targetTag.ftcPose.x, targetTag.ftcPose.y));
            double d = targetTag.ftcPose.y;
            target = new Target(){{
                angle = a;
                distance = d;
            }};
            telemetry.addData("ftcPose", String.format(Locale.US, "%.2f", targetTag.ftcPose.x) + ", "
                    + String.format(Locale.US, "%.2f", targetTag.ftcPose.y));
            telemetry.addLine(target.toString());
        }

        autoMode = gamepad1.yWasReleased() || gamepad2.yWasReleased();
        telemetry.addData("mode", autoMode);

        // launcher controls read from trigger and dPad to determine power apply onto flywheels and turret motors.
        LauncherControls launcherControls = LauncherControls.readControls(gamepadReading);
        // If left bumper if pressed down, we compute the powers to aim, and replace the raw controls.
        if (gamepadReading.leftBumper && target != null)
        {
            double tAngle = launcher.getTurretAngle();
            double[] rpms  = launcher.getWheelPRMs();
            AimingParameters aimingParameters = LauncherControls.computeAimingParameters(target);
            telemetry.addData("target rpm", aimingParameters.launcherRpm);
            telemetry.addData("target angle", aimingParameters.shootingAngle);
            double tRotPower = LauncherControls.computeTurretRotationPower(tAngle, aimingParameters.shootingAngle, 3);
            double[] wheelPowers = LauncherControls.computeShooterWheelPower(rpms, aimingParameters.launcherRpm);
            //launcherControls = new LauncherControls(wheelPowers[0], wheelPowers[1], tRotPower, gamepadReading.aButton);
            //before we install stop limit on turret, let us set turret rotation power to the original
            launcherControls = new LauncherControls(wheelPowers[0], wheelPowers[1],
                    launcherControls.turretPower, gamepadReading.aButton);
        }

        DrivetrainControls driveTrainControls = DrivetrainControls.readControls(gamepadReading);
        // If x - aiming button is down and target visible, combine extra rotation power to aim;
        if (gamepadReading.xButton && target != null){
            double extraRotationPower = DrivetrainControls.computeAimingPower(target.angle, 2);
            telemetry.addData("extraRotationPower", extraRotationPower);
            driveTrainControls.combinePower(0, 0, extraRotationPower);
        }

        double intakePower = gamepadReading.rightBumper ? 1.0 : gamepadReading.rightTrigger;
        ///  Actions
        intake.run(intakePower);

        if (gamepadReading.bWasReleased){
            launcher.abort();
        }
        else
            launcher.run(launcherControls);

        drivetrain.run(driveTrainControls);
    }

}
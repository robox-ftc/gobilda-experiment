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

@TeleOp(name = "StarterBotTeleop-2025Decode", group = "StarterBot")
public class StarterBotTeleop extends OpMode {

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
    private int targetTagId = -1;
    private StartPosition startPosition = StartPosition.TBD;

    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap, telemetry, false);
        launcher = new Launcher(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        telemetry.addData("Status", "Motors Initialized");

        long acquTime = System.nanoTime();
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setCameraPose(
                        new Position(DistanceUnit.INCH, 9, 4, 17, acquTime),
                        new YawPitchRollAngles( AngleUnit.DEGREES,0, 15, 0, acquTime))
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        telemetry.addLine("Camera initialized. Waiting for start...");
        telemetry.update();
    }

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
        }

        if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
            startPosition = StartPosition.NEAR;
            telemetry.addLine("Starting position NEAR.");
        }
    }

    @Override
    public void start() {
        telemetry.addData("Target ID", targetTagId);
        telemetry.addData("Starting from", startPosition);
    }

    @Override
    public void loop() {

        Target target = new Target(80, 0);   // <-- now LOCAL variable inside loop()

        GamePadReadings gamepadReading = new GamePadReadings(){{
            bButton = gamepad1.b || gamepad2.b;
            aButton = gamepad1.a || gamepad2.a;
            xButton = gamepad1.x || gamepad2.x;
            yButton = gamepad1.y || gamepad2.y;
            leftBumper = gamepad1.left_bumper || gamepad2.left_bumper;
            rightBumper = gamepad1.right_bumper || gamepad2.right_bumper;

            leftStickX = Math.abs(gamepad1.left_stick_x) >= Math.abs(gamepad2.left_stick_x) ?
                    gamepad1.left_stick_x : gamepad2.left_stick_x;
            leftStickY = Math.abs(gamepad1.left_stick_y) >= Math.abs(gamepad2.left_stick_y) ?
                    gamepad1.left_stick_y : gamepad2.left_stick_y;
            rightStickX = Math.abs(gamepad1.right_stick_x) >= Math.abs(gamepad2.right_stick_x) ?
                    gamepad1.right_stick_x : gamepad2.right_stick_x;
            rightStickY = Math.abs(gamepad1.right_stick_y) >= Math.abs(gamepad2.right_stick_y) ?
                    gamepad1.right_stick_y : gamepad2.right_stick_y;

            leftTrigger = Math.max(gamepad1.left_trigger, gamepad2.left_trigger);
            rightTrigger = Math.max(gamepad1.right_trigger, gamepad2.right_trigger);

            dPadUp = gamepad1.dpad_up || gamepad2.dpad_up;
            dPadDown = gamepad1.dpad_down || gamepad2.dpad_down;
            dPadLeft = gamepad1.dpad_left || gamepad2.dpad_left;
            dPadRight = gamepad1.dpad_right || gamepad2.dpad_right;
            dPad = dPadDown && dPadUp && dPadLeft && dPadRight;

            aWasReleased = gamepad1.aWasReleased() || gamepad2.aWasReleased();
            aWasPressed  = gamepad1.aWasPressed()  || gamepad2.aWasPressed();
            bWasReleased = gamepad1.bWasReleased() || gamepad2.bWasReleased();
            bWasPressed  = gamepad1.bWasPressed()  || gamepad2.bWasPressed();
            xWasReleased = gamepad1.xWasReleased() || gamepad2.xWasReleased();
            xWasPressed  = gamepad1.xWasPressed()  || gamepad2.xWasPressed();
            yWasReleased = gamepad1.yWasReleased() || gamepad2.yWasReleased();
            yWasPressed  = gamepad1.yWasPressed()  || gamepad2.yWasPressed();
        }};

        AprilTagDetection targetTag = null;

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if ((detection.metadata.id == 20 || detection.metadata.id == 24) &&
                    targetTagId == detection.metadata.id) {
                targetTag = detection;
            }
        }

        if (targetTag != null){
            double a = Math.toDegrees(Math.atan2(targetTag.ftcPose.x, targetTag.ftcPose.y));
            double d = targetTag.ftcPose.y;
            target.angle = a;
            target.distance = d;
            telemetry.addLine(target.toString());
        }

        autoMode = gamepad1.yWasReleased() || gamepad2.yWasReleased();
        telemetry.addData("mode", autoMode);

        LauncherControls launcherControls = LauncherControls.readControls(gamepadReading);

        if (gamepadReading.leftBumper && gamepadReading.rightBumper)
        {
            double tAngle = launcher.getTurretAngle();
            double[] rpms  = launcher.getWheelPRMs();
            AimingParameters aimingParameters = LauncherControls.computeAimingParameters(target);
            telemetry.addData("target rpm", aimingParameters.launcherRpm);
            telemetry.addData("target angle", aimingParameters.shootingAngle);
            double tRotPower = LauncherControls.computeTurretRotationPower(tAngle, aimingParameters.shootingAngle, 3);
            double[] wheelPowers = LauncherControls.computeShooterWheelPower(rpms, aimingParameters.launcherRpm);

            launcherControls = new LauncherControls(
                    wheelPowers[0], wheelPowers[1],
                    launcherControls.turretPower,
                    gamepadReading.aButton
            );

        } else if (gamepadReading.leftBumper) {
            double[] wheelPowers = {0.88, 0.88};
            launcherControls = new LauncherControls(
                    wheelPowers[0], wheelPowers[1],
                    launcherControls.turretPower,
                    gamepadReading.aButton
            );
        } else if (gamepadReading.rightBumper) {
            double[] wheelPowers = {0.73, 0.73};
            launcherControls = new LauncherControls(
                    wheelPowers[0], wheelPowers[1],
                    launcherControls.turretPower,
                    gamepadReading.aButton
            );
        }

        DrivetrainControls driveTrainControls = DrivetrainControls.readControls(gamepadReading);

        if (gamepadReading.xButton && targetTag != null){
            double extraRotationPower = DrivetrainControls.computeAimingPower(target.angle, 2);
            driveTrainControls.combinePower(0, 0, extraRotationPower);
        }

        double intakePower = gamepadReading.rightBumper ? 1.0 : gamepadReading.rightTrigger;
        intake.run(intakePower);

        if (gamepadReading.bWasReleased){
            launcher.abort();
        }
        else launcher.run(launcherControls);

        drivetrain.run(driveTrainControls);
    }
}
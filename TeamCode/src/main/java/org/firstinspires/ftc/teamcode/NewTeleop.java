
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
import org.opencv.core.Mat;

import java.util.List;

@TeleOp(name = "Teleop-2026Decode", group = "StarterBot")
public class NewTeleop extends OpMode {

    public enum StartPosition {
        NEAR,
        FAR
    }
    private Launcher launcher;
    private Drivetrain drivetrain;
    private Intake intake;
    private Turret turret;
    private int targetTagId = 20;
    private Target target; // default value, unchanged
    private StartPosition startPosition = StartPosition.NEAR;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap, telemetry, false);
        launcher = new Launcher(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        telemetry.addData("Status", "Motors Initialized");
        turret = new Turret(hardwareMap, telemetry);
        telemetry.addLine("Camera initialized. Waiting for start...");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        // choose color and location, can be changed if misclicked, default blue near
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

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        if (startPosition == StartPosition.NEAR) {
            // definitely change based on auto
            target = new Target(132, targetTagId == 20 ? Math.toDegrees(Math.atan2(1, 2)) : -Math.toDegrees(Math.atan2(1, 2)));
        } else {
            // definitely change based on auto
            target = new Target(72, 0);
        }
        telemetry.addData("Target ID", targetTagId);
        telemetry.addData("Starting from", startPosition);
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        // Sensing
        // This is the global readings.
        GamePadReadings gamepadReading = new GamePadReadings(){{
            bButton = gamepad1.b || gamepad2.b;
            aButton = gamepad1.a || gamepad2.a;
            xButton = gamepad1.x || gamepad2.x;
            yButton = gamepad1.y || gamepad2.y;
            leftBumper = gamepad1.left_bumper || gamepad2.left_bumper;
            rightBumper = gamepad1.right_bumper || gamepad2.right_bumper;

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
            dPad = dPadUp || dPadDown || dPadLeft || dPadRight;

            aWasReleased = gamepad1.aWasReleased() || gamepad2.aWasReleased();
            aWasPressed = gamepad1.aWasPressed() || gamepad2.aWasPressed();

            bWasReleased = gamepad1.bWasReleased() || gamepad2.bWasReleased();
            bWasPressed = gamepad1.bWasPressed() || gamepad2.bWasPressed();

            xWasReleased = gamepad1.xWasReleased() || gamepad2.xWasReleased();
            xWasPressed = gamepad1.xWasPressed() || gamepad2.xWasPressed();

            yWasReleased = gamepad1.yWasReleased() || gamepad2.yWasReleased();
            yWasPressed = gamepad1.yWasPressed() || gamepad2.yWasPressed();
        }};

        DrivetrainControls driveControls = DrivetrainControls.readControls(gamepadReading);
        LauncherControls launcherControls = LauncherControls.readControls(gamepadReading);
        double intakePower = gamepadReading.rightBumper ? 1.0 : gamepadReading.rightTrigger;
        intake.run(intakePower);
        turret.run(targetTagId, gamepadReading);

        if (gamepadReading.bWasReleased) {
            launcher.abort();
        } else launcher.run(launcherControls);

        drivetrain.run(driveControls);
    }
}
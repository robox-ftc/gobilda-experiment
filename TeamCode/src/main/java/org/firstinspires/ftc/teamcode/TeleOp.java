
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.devices.Drivetrain;
import org.firstinspires.ftc.teamcode.devices.DrivetrainControls;
import org.firstinspires.ftc.teamcode.devices.GamePadReadings;
import org.firstinspires.ftc.teamcode.devices.Intake;
import org.firstinspires.ftc.teamcode.devices.Launcher;
import org.firstinspires.ftc.teamcode.devices.LauncherControls;
import org.firstinspires.ftc.teamcode.devices.Ramp;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.*;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Teleop-2027-Intel", group = "StarterBot")
public class TeleOp extends OpMode {
    private final Set<Integer> redID = new HashSet<>();
    private final Set<Integer> blueID = new HashSet<>();
    private GamePadReadings gamePadReading;
    private Launcher launcher;
    private Drivetrain drivetrain;
    private Intake intake;
    private Ramp ramp;
    private GoBildaPinpointDriver pinpoint;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private final double TOLERANCE = 0.05;
    private final double SHOOTING_FRACTION = 0.5; // a bit lower
    private boolean color = true; // true: blue, false: red
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        for (int i = 0; i < 8; i++) {
            redID.add(i + 30);
            blueID.add(i + 38);
        }
        gamePadReading = new GamePadReadings();
        drivetrain = new Drivetrain(hardwareMap, telemetry, false);
        try {
            launcher = new Launcher(hardwareMap, telemetry);
            intake = new Intake(hardwareMap, telemetry);
            ramp = new Ramp(hardwareMap, telemetry);
            telemetry.addData("Status: ", "Motors Initialized");
        } catch (Exception e) {
            telemetry.addData("Error: ", e.toString());
            telemetry.addLine("Drivetrain-Only Started");
        }
        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD); // Not necessarily correct
            pinpoint.setOffsets(0, 7.5, DistanceUnit.INCH);
            pinpoint.resetPosAndIMU();
            telemetry.addData("Status: ", "Odometry Initialized");
        } catch (Exception e) {
            telemetry.addData("Error: ", e.toString());
            telemetry.addLine("Odometry failed to initialize");
        }
        try {
            // Pasted
            long time = System.nanoTime();
            aprilTag = new AprilTagProcessor.Builder()
                    .setDrawAxes(true)
                    .setDrawCubeProjection(true)
                    .setDrawTagOutline(true)
                    .setCameraPose(
                            new Position(DistanceUnit.INCH, 6, 7, 14, time),
                            new YawPitchRollAngles(AngleUnit.DEGREES, 0, 15, 0, time))
                    .build();
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } catch (Exception e) {
            telemetry.addData("Error: ", e.toString());
            telemetry.addLine("Camera failed to initialize");
        }
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        // choose color and location, can be changed if misclicked, default blue near
        if (gamepad1.xWasPressed() || gamepad2.xWasPressed()) {
            color = true;
            telemetry.addLine("Team BLUE is selected");
        }
        if (gamepad1.bWasPressed() || gamepad2.bWasPressed()){
            color = false;
            telemetry.addLine("Team RED is selected.");
        }
        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        // Sensing
        // This is the global readings.
        double heading = 0;
        try {
            gamePadReading.update(gamepad1, gamepad2);
            heading = pinpoint.getHeading(AngleUnit.DEGREES);
            pinpoint.update();
        } catch (Exception ignored) {}
        double x = getLocation();
        DrivetrainControls driveControls = DrivetrainControls.readControls(gamePadReading);
        LauncherControls launcherControls = LauncherControls.readControls(gamePadReading);
        double intakePower = gamePadReading.bButton ? -0.5 : gamePadReading.rightTrigger;
        if (gamePadReading.leftBumper) {
            launcherControls.wheelPower = 1;
            if (launcher.getLeftWheelRPM() / Launcher.LAUNCHER_MAX_VELOCITY_RPM > SHOOTING_FRACTION) {
                launcherControls.wheelPower = SHOOTING_FRACTION;
                launcherControls.rampPower = 1;
                intakePower = 1;
            } else {
                driveControls.translationXPower = -clamp(x * TOLERANCE);
            }
            ramp.run(launcherControls);
            launcher.run(launcherControls);
        } else {
            launcher.run(launcherControls);
            ramp.run(launcherControls);
        }
        intake.run(intakePower);
        drivetrain.run(driveControls);
        if (pinpoint != null) {
            telemetry.addData("X", pinpoint.getPosX(DistanceUnit.INCH));
            telemetry.addData("Y", pinpoint.getPosY(DistanceUnit.INCH));
            telemetry.addData("Heading", heading);
        }
        telemetry.update();
    }

    private double getLocation() {
        try {
            ArrayList<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata == null) {
                    continue;
                }
                if ((redID.contains(detection.metadata.id) && !color) || (blueID.contains(detection.metadata.id) && color)) {
                    return detection.ftcPose.x;
                }
            }
        } catch (Exception ignored) {}
        return 0;
    }

    public double clamp(double x) {
        if (Math.abs(x) < TOLERANCE) {
            return 0;
        }
        return Math.max(Math.min(x, 1), -1);
    }
}
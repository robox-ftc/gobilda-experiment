package org.firstinspires.ftc.teamcode.devices;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.data.Target;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class Turret {
    private final DcMotorEx wheel;
    // private CRServo wheel;
    private final AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private final Telemetry telemetry;
    private boolean auto; // run automatically

    private Target target;

    // arbitrary (tunable) constants
    private static final double TICKS_PER_SEC = 154.1 * 1150.0 / 60.0;
    private static final double MANUAL_ROTATION_SPEED = TICKS_PER_SEC / 3.0;
    private static final double AUTO_ROTATION_SPEED = TICKS_PER_SEC / 22.5;
    // 70 degree FOV, range for "close (within p)" is 45 degrees

    /* P[I]D
    private static final double P = 1;
    private static final double D = 0.1;
     */

    public Turret(HardwareMap hardwareMap, Telemetry telemetry) {
        wheel = hardwareMap.get(DcMotorEx.class, "turret");
        wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheel.setDirection(DcMotorSimple.Direction.FORWARD);
        wheel.setZeroPowerBehavior(BRAKE);

        target = new Target();

        // Initialize AprilTag processor, the same as previous
        long time = System.nanoTime();
        aprilTag = new AprilTagProcessor.Builder().setDrawAxes(true).setDrawCubeProjection(true).setDrawTagOutline(true).setCameraPose(
                        new Position(DistanceUnit.INCH, 9, 4, 17, time),
                        new YawPitchRollAngles( AngleUnit.DEGREES,0, 15, 0, time))
                .build();

        // Create vision portal using the built-in webcam
        visionPortal = new VisionPortal.Builder().setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")).addProcessor(aprilTag).build();

        this.telemetry = telemetry;
        this.auto = true;
    }

    public void run(int targetId) {
        // Auto
        AprilTagDetection targetTag = getAprilTag(targetId);
        if (targetTag != null) {
            target.angle = Math.toDegrees(Math.atan2(targetTag.ftcPose.x, targetTag.ftcPose.y));
        }
        telemetry.addLine(target.toString());
        wheel.setVelocity(pClamp(target.angle));
    }

    public void run(int targetId, GamePadReadings reading) {
        // TeleOp
        AprilTagDetection targetTag = getAprilTag(targetId);
        if (reading.backWasPressed) {
            auto = !auto;
        }
        if (reading.dPad || targetTag == null || !auto) {
            // manual rotation, DO NOT PRESS D-PAD ON ACCIDENT
            int rotation = (reading.dPadLeft ? -1 : 0) + (reading.dPadRight ? 1 : 0);
            wheel.setVelocity(rotation * MANUAL_ROTATION_SPEED);
        } else {
            // execute if turret spinning during launching is bad
            // if (reading.leftBumper && reading.leftTrigger > 0) {}
            target.angle = Math.toDegrees(Math.atan2(targetTag.ftcPose.x, targetTag.ftcPose.y));
            telemetry.addLine(target.toString());
            wheel.setVelocity(pClamp(target.angle));
        }
    }

    public AprilTagDetection getAprilTag(int targetId) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && detection.metadata.id == targetId) {
                return detection;
            }
        }
        telemetry.addLine("UNABLE TO DETECT APRILTAG");
        return null;
    }


    private double pClamp(double angle) {
        return Math.min(Math.max(angle * AUTO_ROTATION_SPEED, -TICKS_PER_SEC), TICKS_PER_SEC);
    }

//    private double PID(double angle, double prev) {
//        return Math.min(TICKS_PER_SEC, Math.abs(angle) > 45 ? angle * AUTO_ROTATION_SPEED : (angle * P + (prev - angle) * D) * AUTO_ROTATION_SPEED);
//    }
}
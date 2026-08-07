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
    private boolean auto; // run automatically or not

    private Target target;
    private static final double TICKS_PER_SEC = 145.1 * 1150.0 / 60.0;
    private static final double TICKS_PER_DEG = 145.1 / 360;
    private static final double RATIO = 7.7 / 1.56; // ratio of the pulley on the turret

    // arbitrary (tunable) constants
    private static final double TOLERANCE = 0.1; // might be useless
    private static final double MANUAL_ROTATION_SPEED = TICKS_PER_SEC / 3.0;
    private static final double AUTO_ROTATION_FACTOR = TICKS_PER_DEG * RATIO;

    /* P[I]D
    private static final double P = 1;
    private static final double D = 0.1;
     */

    public Turret(HardwareMap hardwareMap, Telemetry telemetry) {
        wheel = hardwareMap.get(DcMotorEx.class, "turret");
        wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheel.setDirection(DcMotorSimple.Direction.FORWARD);
        wheel.setZeroPowerBehavior(BRAKE);

        target = new Target();

        // Initialize AprilTag processor, the same as previous
        long time = System.nanoTime();
        aprilTag = new AprilTagProcessor.Builder().setDrawAxes(true).setDrawCubeProjection(true).setDrawTagOutline(true).setCameraPose(
                        new Position(DistanceUnit.INCH, 9, 5.5, 12, time),
                        new YawPitchRollAngles(AngleUnit.DEGREES,0, 15, 0, time))
                .build();

        // Create vision portal using the built-in webcam
        visionPortal = new VisionPortal.Builder().setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")).addProcessor(aprilTag).build();

        this.telemetry = telemetry;
        this.auto = true;
    }
    /**
     * True only when the camera is actually streaming. The VisionPortal can be
     * built successfully and still end up in ERROR (webcam unplugged, USB fault),
     * in which case there are no detections and auto-aim would swing the turret
     * to a stale or zeroed target.
     */
    public boolean hasCamera() {
        return visionPortal != null && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING;
    }

    public void setTarget(int targetID, double heading) {
        // Auto
        if (!hasCamera()) {
            telemetry.addLine("NO CAMERA - cannot acquire target");
            return;
        }
        AprilTagDetection targetTag = getAprilTag(targetID);
        if (targetTag != null) {
            target.angle = Math.toDegrees(Math.atan2(targetTag.ftcPose.x, targetTag.ftcPose.y));
            target.heading = heading;
        }
        telemetry.addData("target heading: ", target.angle);
        telemetry.addData("current heading: ", target.heading);
    }

    public void run(double angle) {
        // Auto
        double error = clamp(angle - target.angle - target.heading);
        wheel.setTargetPosition(angleToPosition(error));
        if (wheel.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            wheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        wheel.setVelocity(TICKS_PER_SEC);
    }

    public void run(int targetId, double angle, GamePadReadings reading) {
        // TeleOp
        boolean cameraOk = hasCamera();
        if (!cameraOk && auto) {
            // No camera means no detections, so auto-aim can only chase a stale
            // target. Drop to manual and leave it there until the driver opts in.
            auto = false;
            if (wheel.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                wheel.setVelocity(0);
            }
        }
        if (reading.backWasPressed) {
            auto = cameraOk && !auto;
            if (!auto && wheel.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            telemetry.addData("Auto-align: ", auto);
        }
        telemetry.addData("Camera", cameraOk ? "streaming" : "UNAVAILABLE - manual turret only");
        if (reading.xWasPressed) { // Find Target
            setTarget(targetId, angle);
        }
        if (auto && (reading.leftTrigger > TOLERANCE || reading.aWasPressed || reading.yWasPressed || reading.bWasPressed)) {
            telemetry.addLine(target.toString());
            // Auto
            double error = clamp(angle - target.angle - target.heading);
            wheel.setTargetPosition(angleToPosition(error));
            if (wheel.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                wheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            wheel.setVelocity(TICKS_PER_SEC);
        } else if (reading.dPad) {
            // manual rotation, DO NOT PRESS D-PAD ON ACCIDENT OR IT'LL BREAK
            int rotation = (reading.dPadLeft ? -1 : 0) + (reading.dPadRight ? 1 : 0);
            if (wheel.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            wheel.setVelocity(rotation * MANUAL_ROTATION_SPEED);
        } else if (!auto && wheel.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
            wheel.setVelocity(0);
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

    public void resetTurret() {
        wheel.setTargetPosition(0);
        if (wheel.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            wheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        wheel.setVelocity(TICKS_PER_SEC);
        telemetry.addLine("Turret Reset");
    }
    private double clamp(double angle) {
        if (angle > 180) angle -= 360;
        if (angle < -180) angle += 360;
        return angle;
    }

    private int angleToPosition(double angle) {
        return (int) (angle * AUTO_ROTATION_FACTOR);
    }


//    private double PID(double angle, double prev) {
//        return Math.min(TICKS_PER_SEC, Math.abs(angle) > 45 ? angle * AUTO_ROTATION_SPEED : (angle * P + (prev - angle) * D) * AUTO_ROTATION_SPEED);
//    }
}
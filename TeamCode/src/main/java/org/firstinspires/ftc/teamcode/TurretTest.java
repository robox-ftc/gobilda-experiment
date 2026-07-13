package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import android.graphics.MaskFilter;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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

@Autonomous(name = "Turret", group = "Robot")
public class TurretTest extends OpMode {
    private DcMotorEx wheel;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private Target target;
    private static final double TICKS_PER_SEC = 751.8 * 223 / 60.0; // Slowdown by 5x to use, like 751.8 * 223 / 60.0 / 5.0, otherwise wires might get tangled in 0.25 secs, be careful
    private static final double ROTATION_SPEED = TICKS_PER_SEC / 15; // Arbitrary
    private static final double TOLERANCE = 1.0;
    @Override
    public void init() {
        wheel = hardwareMap.get(DcMotorEx.class, "turret");
        wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheel.setDirection(DcMotorSimple.Direction.FORWARD);
        wheel.setZeroPowerBehavior(BRAKE);

        target = new Target();

        // Initialize AprilTag processor, the same as previous
        long time = System.nanoTime();
        aprilTag = new AprilTagProcessor.Builder().setDrawAxes(true).setDrawCubeProjection(true).setDrawTagOutline(true).setCameraPose(
                        new Position(DistanceUnit.INCH, 6, 0, 0, time),
                        new YawPitchRollAngles(AngleUnit.DEGREES,0, 0, 0, time))
                .build();

        // Create vision portal using the built-in webcam
        visionPortal = new VisionPortal.Builder().setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")).addProcessor(aprilTag).build();
    }

    @Override
    public void loop() {
            // Auto
        AprilTagDetection targetTag = getAprilTag(20);
        if (targetTag != null) {
            target.angle = Math.toDegrees(Math.atan2(targetTag.ftcPose.x, targetTag.ftcPose.y));
            telemetry.addLine(target.toString());
        }
        if (Math.abs(target.angle) > TOLERANCE) {
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
        return Math.min(Math.max(angle * ROTATION_SPEED, -TICKS_PER_SEC), TICKS_PER_SEC);
    }
}

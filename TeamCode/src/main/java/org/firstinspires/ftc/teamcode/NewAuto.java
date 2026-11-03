package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.devices.Drivetrain;
import org.firstinspires.ftc.teamcode.devices.DrivetrainControls;
import org.firstinspires.ftc.teamcode.devices.Intake;
import org.firstinspires.ftc.teamcode.devices.Launcher;
import org.firstinspires.ftc.teamcode.devices.Ramp;
import org.firstinspires.ftc.teamcode.devices.Task;
import org.firstinspires.ftc.teamcode.utils.CellTags;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.*;

@Autonomous(name = "Auto-2027", group = "Robot")
public class NewAuto extends OpMode {

    public enum Mode {
        LEAVE,
        PARK,
        SHOOT,
        INTAKE
    }
    private static final double TOLERANCE = 1; // Arbitrary
    private static final int THRESHOLD = 10000; // Milliseconds before it checks the camera
    private static final double RTOLERANCE = 5; // Arbitrary

    // Tunable P
    private final double P = 1.0 / 20.0;
    // Rotation Factor
    private final double R = 2 * 12.0 / 360.0;
    private boolean color = true; // default blue

    private Mode mode;
    private GoBildaPinpointDriver pinpoint;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private Drivetrain drivetrain;
    private Intake intake;
    private Ramp ramp;
    private Launcher launcher;
    private DrivetrainControls controls;
    private Queue<Task> queue;
    private ElapsedTime timer;

    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap, telemetry, false);
        controls = new DrivetrainControls();
        telemetry.addLine("Drivetrain initialized");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(0, 7.5, DistanceUnit.INCH);
        pinpoint.resetPosAndIMU();
        try {
            launcher = new Launcher(hardwareMap, telemetry);
            telemetry.addLine("Launcher initialized");
            intake = new Intake(hardwareMap, telemetry);
            ramp = new Ramp(hardwareMap, telemetry);
            telemetry.addLine("Intake initialized");
        } catch (Exception e) {
            launcher = null;
            intake = null;
            ramp = null;
            telemetry.addLine("Drivetrain-only started");
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
        mode = Mode.PARK;
        queue = new LinkedList<>();
        timer = new ElapsedTime();
        telemetry.update();
    }

    @Override
    public void init_loop() {
        if (gamepad1.xWasPressed() || gamepad2.xWasPressed()) {
            color = true;
            telemetry.addLine("Team BLUE is selected");
        }
        if (gamepad1.bWasPressed() || gamepad2.bWasPressed()) {
            color = false;
            telemetry.addLine("Team RED is selected");
        }
        if (gamepad1.yWasPressed() || gamepad2.yWasPressed()) {
            telemetry.addLine("Shooting Auto is selected");
            mode = Mode.SHOOT;
        }
        if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
            telemetry.addLine("Parking Auto is selected");
            mode = Mode.PARK;
        }
        // Toggle auto
        if (gamepad1.backWasPressed() || gamepad2.backWasPressed()) {
            switch (mode) {
                case PARK:
                    mode = Mode.LEAVE;
                    telemetry.addLine("Leave Auto is selected");
                case LEAVE:
                    mode = Mode.SHOOT;
                    telemetry.addLine("Parking Auto is selected");
                case SHOOT:
                    mode = Mode.INTAKE;
                    telemetry.addLine("Shoot-Intake Auto is selected");
                case INTAKE:
                    mode = Mode.PARK;
                    telemetry.addLine("Shooting Auto is selected");
            }
        }
        telemetry.update();
    }

    @Override
    public void start() {
        switch (mode) {
            case LEAVE:
                queue.add(new Task(1000, 2000, Task.Type.TRANSLATE, 0, 0.5));
                break;
            case PARK:
                queue.add(new Task(1000, 2000, Task.Type.TRANSLATE, 0, 0.5));
                queue.add(new Task(11000, 12000, Task.Type.TRANSLATE, 0, -0.35));
                break;
            case SHOOT:
                queue.add(new Task(0, 5000, Task.Type.LAUNCH, 0.8, 2));
                queue.add(new Task(5500, 6500, Task.Type.TRANSLATE, -1, 0.75));
                queue.add(new Task(7000, 8500, Task.Type.TRANSLATE, 0, 1));
                break;
            case INTAKE:
                queue.add(new Task(0, 5000, Task.Type.LAUNCH, 0.8, 2));
                queue.add(new Task(5500, 6000, Task.Type.ROTATE, 0.1, 0));
                queue.add(new Task(5500, 7000, Task.Type.TRANSLATE, -0.75, -0.2));
                queue.add(new Task(7500, 8000, Task.Type.ROTATE, -0.1, 0));
                queue.add(new Task(8500, 9000, Task.Type.TRANSLATE, 0.75, 0.2));
                queue.add(new Task(9250, 9750, Task.Type.TRANSLATE, 0, -0.3));
                queue.add(new Task(10000, 15000, Task.Type.LAUNCH, 0.8, 2));
                queue.add(new Task(15500, 16500, Task.Type.TRANSLATE, -1, 0.75));
                queue.add(new Task(17000, 18500, Task.Type.TRANSLATE, 0, 1));
                break;
        }
        timer.reset();
    }

    @Override
    public void loop() {
        controls.translationXPower = 0;
        controls.translationYPower = 0;
        controls.rotationPower = 0;

        pinpoint.update();
        double x = pinpoint.getPosX(DistanceUnit.INCH);
        double y = pinpoint.getPosY(DistanceUnit.INCH);
        double heading = pinpoint.getHeading(AngleUnit.DEGREES);

        if (!queue.isEmpty()) {
            Task task = queue.peek();

            switch (task.type) {
                case TRANSLATE:
                    double errorY = task.target - x;
                    double errorX = task.target2 - y;
                    // square distance, unlikely it'll reach the end of the task
                    if (Math.hypot(errorX, errorY) <= TOLERANCE || timer.milliseconds() > task.end) {
                        queue.remove();
                    } else {
                        controls.translationXPower = pClamp(errorX);
                        controls.translationYPower = pClamp(errorY);
                    }
                    if (intake != null) {
                        intake.spin(1);
                    }
                    break;
                case ROTATE:
                    double error = task.target - heading;
                    if (error > 180) error -= 360;
                    if (error < -180) error += 360;
                    // Try to avoid the robot turning more than 360
                    if (Math.abs(error) <= RTOLERANCE || timer.milliseconds() > task.end) {
                        queue.remove();
                    } else {
                        controls.rotationPower = pClamp(error * R);
                    }
                    if (intake != null) {
                        intake.spin(0);
                    }
                    break;
                case LAUNCH:
                    if (timer.milliseconds() > THRESHOLD && aprilTag != null) {
                        AprilTagDetection target = null;
                        try {
                            ArrayList<AprilTagDetection> currentDetections = aprilTag.getDetections();
                            for (AprilTagDetection detection : currentDetections) {
                                // Either of our alliance's CELLs, as a cluster or a single tag.
                                if (CellTags.isOurs(detection, color)) {
                                    target = detection;
                                    break;
                                }
                            }
                        } catch (Exception ignored) {}
                        if (target == null) {
                            break;
                        }
                    }
                    if (intake != null && launcher != null) {
                        int phase = launcherPhase(task, timer.milliseconds());
                        if (phase == 0) {
                            launcher.spinToFraction(task.target);
                            ramp.spin(0);
                        } else if (phase <= task.target2) {
                            launcher.spinToFraction(task.target);
                            ramp.spin(1);
                        } else {
                            launcher.abort();
                            queue.remove();
                        }
                    }
                    break;
                default:
                    controls.translationXPower = 0;
                    controls.translationYPower = 0;
                    controls.rotationPower = 0;
                    break;
            }
        } else {
            if (intake != null) {
                intake.spin(0);
            }
            if (launcher != null) {
                launcher.abort();
            }
        }
        if (drivetrain != null) drivetrain.run(controls);

        telemetry.addData("Current X", x);
        telemetry.addData("Current Y", y);
        telemetry.addData("Current Heading", heading);
        telemetry.addData("Remaining Tasks", queue.size());
        telemetry.update();
    }

    private int launcherPhase(Task task, double time) {
        try {
            int begin = task.begin;
            int end = task.end;
            int shots = (int) (task.target2) + 1; // +1 because of time needed to spin up, could be an int also
            return Math.min(shots, (int) ((time - begin) * shots / (end - begin)));
        } catch (Exception e) {
            return (int) (task.target2 + 1); // end
        }
    }

    private double pClamp(double error) {
        return Math.min(Math.max(error * P, -1), 1);
    }
}
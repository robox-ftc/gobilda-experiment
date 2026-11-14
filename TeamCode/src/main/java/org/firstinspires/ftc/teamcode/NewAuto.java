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
    private final double P = 1.0 / 100.0; // tuned so 100 inches away would lead to 100 percent power
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
    private final Set<Integer> redID = new HashSet<>();
    private final Set<Integer> blueID = new HashSet<>();

    @Override
    public void init() {
        for (int i = 34; i < 38; i++) {
            redID.add(i);
        }
        for (int i = 42; i < 46; i++) {
            blueID.add(i);
        }
        drivetrain = new Drivetrain(hardwareMap, telemetry, false);
        controls = new DrivetrainControls();
        telemetry.addLine("Drivetrain initialized");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(-5, 5, DistanceUnit.INCH);
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
                            new Position(DistanceUnit.INCH, 0, 4, 7, time),
                            new YawPitchRollAngles(AngleUnit.DEGREES, 0, 50, 0, time))
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
        telemetry.addLine("Y for shooting auto, A for parking auto");
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
                    break;
                case LEAVE:
                    mode = Mode.SHOOT;
                    telemetry.addLine("Parking Auto is selected");
                    break;
                case SHOOT:
                    mode = Mode.INTAKE;
                    telemetry.addLine("Shoot-Intake Auto is selected");
                    break;
                case INTAKE:
                    mode = Mode.PARK;
                    telemetry.addLine("Shooting Auto is selected");
                    break;
            }
        }
        telemetry.update();
    }

    @Override
    public void start() {
        switch (mode) {
            case LEAVE:
                queue.add(new Task(1000, 2000, Task.Type.TRANSLATE, 0, 10)); // 7 inches to go forward
                break;
            case PARK:
                queue.add(new Task(1000, 2000, Task.Type.TRANSLATE, 0, 10));
                queue.add(new Task(11000, 12000, Task.Type.TRANSLATE, 0, -10));
                break;
            case SHOOT:
                queue.add(new Task(0, 1000, Task.Type.TRANSLATE, 0, 7)); // 7 inches forward
                queue.add(new Task(1000, 6000, Task.Type.LAUNCH, 0.7, 2));
                queue.add(new Task(6500, 7500, Task.Type.TRANSLATE, -15, 15)); // -20, 15 is 25 inches left and 15 inches forward
                queue.add(new Task(9000, 10000, Task.Type.TRANSLATE, -10, 65)); // -30, 70 is 35 inches left and 70 inches forward
                queue.add(new Task(10000, 11000, Task.Type.TRANSLATE, -25, 70)); // -35, 70 is 45 inches left and 70 inches forward
                queue.add(new Task(14000, 15000, Task.Type.TRANSLATE, -30, 70)); // -40, 70 is 50 inches left and 70 inches forward
                break;
            case INTAKE:
                queue.add(new Task(0, 1000, Task.Type.TRANSLATE, 0, 7)); // 10 inches forward
                queue.add(new Task(1000, 6000, Task.Type.LAUNCH, 0.7, 2));
                queue.add(new Task(6500, 7500, Task.Type.TRANSLATE, -30, -5));
                queue.add(new Task(8500, 9500, Task.Type.TRANSLATE, 5, -5));
                queue.add(new Task(10000, 15000, Task.Type.LAUNCH, 0.7, 2));
                queue.add(new Task(15500, 16500, Task.Type.TRANSLATE, -15, 15)); // -15, 15 is 20 inches left and 15 inches forward
                queue.add(new Task(17000, 18000, Task.Type.TRANSLATE, -15, 60)); // -15, 60 is 20 inches left and 75 inches forward
                queue.add(new Task(18500, 19000, Task.Type.TRANSLATE, -35, 70)); // -35, 85 is 35 inches left and 55                queue.add(new Task(15500, 16500, Task.Type.TRANSLATE, -15, 15)); // -15, 15 is 20 inches left and 15 inches forward
                queue.add(new Task(17000, 18000, Task.Type.TRANSLATE, -15, 60)); // -15, 60 is 20 inches left and 75 inches forward
                queue.add(new Task(18500, 19000, Task.Type.TRANSLATE, -35, 70)); // -35, 85 is 35 inches left and 55 inches forward
                break;
        }
        timer.reset();
        pinpoint.resetPosAndIMU();
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
            if (timer.milliseconds() >= task.begin) {
                switch (task.type) {
                    case TRANSLATE:
                        double errorX = x + task.target; // Corrected logic
                        double errorY = y + task.target2;
                        // square distance, unlikely it'll reach the end of the task
                        if (Math.hypot(errorX, errorY) <= TOLERANCE || timer.milliseconds() > task.end) {
                            controls.translationXPower = 0;
                            controls.translationYPower = 0;
                            queue.remove();
                        } else {
                            controls.translationXPower = clamp(errorX);
                            controls.translationYPower = clamp(errorY);
                        }
                        if (intake != null) {
                            intake.spin(1);
                        }
                        break;
                    case ROTATE:
                        double error = task.target + heading;
                        if (error > 180) error -= 360;
                        if (error < -180) error += 360;
                        // Try to avoid the robot turning more than 360
                        if (Math.abs(error) <= RTOLERANCE || timer.milliseconds() > task.end) {
                            controls.rotationPower = 0;
                            queue.remove();
                        } else {
                            controls.rotationPower = clamp(error * R);
                        }
                        if (intake != null) {
                            intake.spin(0);
                        }
                        break;
                    case LAUNCH:
                        if (timer.milliseconds() > task.end) {
                            queue.remove();
                            break;
                        }
                        if (timer.milliseconds() > THRESHOLD && aprilTag != null) {
                            AprilTagDetection target = null;
                            try {
                                ArrayList<AprilTagDetection> currentDetections = aprilTag.getDetections();
                                for (AprilTagDetection detection : currentDetections) {
                                    if (detection.ftcPose != null) {
                                        target = detection;
                                        break;
                                    }
                                }
                            } catch (Exception ignored) {}
                            if (target == null) { // extra, redundant but useful in case code changes
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
                            }
                        } else {
                            queue.remove();
                        }
                        break;
                    default:
                        controls.translationXPower = 0;
                        controls.translationYPower = 0;
                        controls.rotationPower = 0;
                        break;
                }
            }
        } else {
            controls.translationXPower = 0;
            controls.translationYPower = 0;
            controls.rotationPower = 0;
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

    private double clamp(double error) {
        return Math.min(Math.max(error * P, -0.5), 0.5);
    }
}
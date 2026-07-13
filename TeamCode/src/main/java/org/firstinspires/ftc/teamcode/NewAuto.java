package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.devices.Drivetrain;
import org.firstinspires.ftc.teamcode.devices.DrivetrainControls;
import org.firstinspires.ftc.teamcode.devices.Intake;
import org.firstinspires.ftc.teamcode.devices.Launcher;
import org.firstinspires.ftc.teamcode.devices.Task;
import org.firstinspires.ftc.teamcode.devices.Turret;

import java.util.*;

@Autonomous(name = "Auto-2026Decode", group = "Robot")
public class NewAuto extends OpMode {
    public static final int RED = -1;
    public static final int BLUE = 1;
    private static final double TOLERANCE = 1; // Arbitrary
    private static final double RTOLERANCE = 5; // Arbitrary

    // Tunable P
    private final double P = 1.0 / 20.0;
    // Rotation Factor
    private final double R = 2 * 12.0 / 360.0;

    private GoBildaPinpointDriver pinpoint;
    private Drivetrain drivetrain;
    private Intake intake;
    private Launcher launcher;
    private Turret turret;
    private DrivetrainControls controls;
    private int color = BLUE;
    private int aprilTag = 20;
    private boolean near;
    private boolean stay;
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
            telemetry.addLine("Intake initialized");
        } catch (Exception e) {
            launcher = null;
            intake = null;
            telemetry.addLine("Drivetrain-only started");
        }
        try {
            turret = new Turret(hardwareMap, telemetry);
            telemetry.addLine("Camera initialized");
        } catch (Exception e) {
            turret = null;
            telemetry.addLine("No-turret started");
        }
        color = BLUE;
        near = true;
        queue = new LinkedList<>();
        timer = new ElapsedTime();
        telemetry.update();
    }

    @Override
    public void init_loop() {
        if (gamepad1.xWasPressed() || gamepad2.xWasPressed()) {
            telemetry.addLine("Team BLUE is selected");
            aprilTag = 20;
            color = BLUE;
        }
        if (gamepad1.bWasPressed() || gamepad2.bWasPressed()) {
            telemetry.addLine("Team RED is selected");
            aprilTag = 24;
            color = RED;
        }
        if (gamepad1.yWasPressed() || gamepad2.yWasPressed()) {
            telemetry.addLine("Near Position is selected");
            near = true;
        }
        if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
            telemetry.addLine("Far Position is selected");
            near = false;
        }
        if (gamepad1.backWasPressed() || gamepad2.backWasPressed()) {
            telemetry.addData("Stay Auto is selected to be: ", stay);
            stay = !stay;
        }
        telemetry.update();
    }

    @Override
    public void start() {
        if (stay) {
            queue.add(new Task(0, 5000, Task.Type.LAUNCH, 0.75, 3));
        } else if (near) {
            // no ROTATE immediately before LAUNCH
            queue.add(new Task(0, 5000, Task.Type.LAUNCH, 0.75, 3));
            queue.add(new Task(5000, 5500, Task.Type.TRANSLATE, 0, -12));
            queue.add(new Task(10000, 10500, Task.Type.TRANSLATE, -12, -24));
            queue.add(new Task(12000, 12500, Task.Type.TRANSLATE, -15, -28));
            queue.add(new Task(20000, 21000, Task.Type.TRANSLATE, -14, -26));
        } else {
            queue.add(new Task(0, 5000, Task.Type.LAUNCH, 1, 3));
            queue.add(new Task(10000, 11000, Task.Type.TRANSLATE, -12, 6));
            queue.add(new Task(20000, 21000, Task.Type.TRANSLATE, -18, 12));
        }
        if (turret != null && pinpoint != null) {
            pinpoint.update();
            turret.setTarget(aprilTag, pinpoint.getHeading(AngleUnit.DEGREES));
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
                    double errorX = task.target * color - x;
                    double errorY = task.target2 - y;
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
                    double error = task.target * color - heading;
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
                    if (intake != null && launcher != null) {
                        int phase = launcherPhase(task, timer.milliseconds());
                        if (phase == 0) {
//                            if (turret != null) turret.run(heading);
                            launcher.resetFeeder();
                            launcher.spin(task.target);
                        } else if (phase <= task.target2) {
                            launcher.fire();
                            intake.spin(1);
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
            if (turret != null) {
                turret.resetTurret();
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
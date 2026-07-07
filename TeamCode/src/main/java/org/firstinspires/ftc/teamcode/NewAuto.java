package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.devices.Drivetrain;
import org.firstinspires.ftc.teamcode.devices.DrivetrainControls;
import org.firstinspires.ftc.teamcode.devices.Intake;
import org.firstinspires.ftc.teamcode.devices.Launcher;
import org.firstinspires.ftc.teamcode.devices.Task;
import org.firstinspires.ftc.teamcode.devices.Turret;

import java.util.*;

@Autonomous(name = "Teleop-2026Decode-Odo", group = "Robot")
public class NewAuto extends OpMode {
    public static final int RED = -1;
    public static final int BLUE = 1;
    private static final double TOLERANCE = 0.5; // Arbitrary

    private Drivetrain drivetrain;
    private Intake intake;
    private Launcher launcher;
    private Turret turret;
    private DrivetrainControls controls;

    private int color = BLUE;
    private int aprilTag = 20;
    private boolean near;

    // Using a standard Queue instead of a time-based schedule loop
    private Queue<Task> queue;
    private ElapsedTime timer; // Used for non-movement tasks like launching

    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap, telemetry, false);
        controls = new DrivetrainControls();
        turret = new Turret(hardwareMap, telemetry);

        telemetry.addLine("Drivetrain initialized");
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

        color = BLUE;
        near = true;
        queue = new LinkedList<>();
        timer = new ElapsedTime();
    }

    @Override
    public void init_loop() {
        if (gamepad1.xWasPressed() || gamepad2.xWasPressed()) {
            telemetry.addLine("Team BLUE is selected");
            color = BLUE;
        }
        if (gamepad1.bWasPressed() || gamepad2.bWasPressed()) {
            telemetry.addLine("Team RED is selected");
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
    }

    @Override
    public void start() {
        if (near) {
            // no ROTATE immediately before LAUNCH
            queue.add(new Task(0, 4500, Task.LAUNCH, 0.75));
            queue.add(new Task(4500, 5000, Task.TRANSLATE, 24.0, 0.5));
            queue.add(new Task(5000, 5500, Task.ROTATE, 0.5));
            queue.add(new Task(10000, 10500, Task.TRANSLATE, 24.0, 0.5));
            queue.add(new Task(12000, 12500, Task.TRANSLATE, 24.0, 0.5));
            queue.add(new Task(20000, 21000, Task.TRANSLATE, 24.0, 0.5));
        } else {
            queue.add(new Task(0, 5000, Task.LAUNCH, 1));
            queue.add(new Task(10000, 11000, Task.TRANSLATE, 24.0, 0.5));
            queue.add(new Task(20000, 21000, Task.TRANSLATE, 24.0, 0.5));
        }
        timer.reset();
    }

    @Override
    public void loop() {
        controls.translationYPower = 0;
        controls.rotationPower = 0;

//        double x = odometry.getOdometryX(); "magic function"
//        double y = odometry.getOdometryY();
//        double heading = odometry.getHeading();

        if (!queue.isEmpty()) {
            Task task = queue.peek();

            switch (task.type) {
                case Task.TRANSLATE:
//                    double error = task.target - currentY;
//                    if (Math.abs(error) <= TOLERANCE) {
//                        queue.remove();
//                    } else {
//                        controls.translationYPower = error;
//                    }
//                    break;
                case Task.ROTATE:
//                    double error = task.target - heading;
//                    if (Math.abs(error) <= TOLERANCE) {
//                        queue.remove();
//                    } else {
//                        controls.rotationPower = error;
//                    }
//                    break;
                case Task.LAUNCH:
//                    if (!task.executed) {
//                        launcher.resetFeeder();
//                        launcher.spinToVelocity(task.target);
//                        task.executed = true;
//                        timer.reset(); // Track time duration specifically for non-spatial events
//                    } else {
//                        launcher.fire(1);
//                        // task.TOLERANCE can store duration limits for systemic actions
//                        if (timer.milliseconds() >= TOLERANCE) {
//                            launcher.fire(0);
//                            launcher.spinToVelocity(0);
//                            queue.remove();
//                        }
//                    }
//                    break;
            }
        }

        intake.spin(1);
        turret.run(aprilTag);
        drivetrain.run(controls);
//
//        // Telemetry Updates
//        telemetry.addData("Current Y", y);
//        telemetry.addData("Current Heading", h);
//        telemetry.addData("Remaining Tasks", queue.size());
//        telemetry.update();
    }
}
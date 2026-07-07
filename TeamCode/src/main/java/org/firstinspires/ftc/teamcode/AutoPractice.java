
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.devices.Drivetrain;
import org.firstinspires.ftc.teamcode.devices.DrivetrainControls;
import org.firstinspires.ftc.teamcode.devices.Intake;
import org.firstinspires.ftc.teamcode.devices.Launcher;
import org.firstinspires.ftc.teamcode.devices.Task;
import org.firstinspires.ftc.teamcode.devices.Turret;

import java.util.*;

@Autonomous(name = "AutoPractice", group = "Robot")

public class AutoPractice extends OpMode {
    public static final int RED = -1;
    public static final int BLUE = 1;
    private Drivetrain drivetrain;
    private Intake intake;
    private Launcher launcher;
    private Turret turret;
    private DrivetrainControls controls;
    private int color;
    private boolean near;
    private LinkedList<Task> queue;
    private ElapsedTime timer;

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
        color = BLUE; // default is blue
        near = true; // default is near (front)
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
        // add tasks
        if (near) { // near (we aren't using them now)
            queue.add(new Task(0, 2000, Task.TRANSLATE, 1)); // we will use target as magnitude for now
            queue.add(new Task(1000, 2000, Task.ROTATE, 0.5));
            queue.add(new Task(2000, 5000, Task.LAUNCH, 2000)); // this is just for future integration
            queue.add(new Task(5000, 6000, Task.ROTATE, -0.5));
            queue.add(new Task(5000, 7000, Task.TRANSLATE, -0.5));
        } else { // far (we aren't using them now)
            queue.add(new Task(0, 2000, Task.TRANSLATE, 1)); // we will use target as magnitude for now
            queue.add(new Task(1000, 2000, Task.ROTATE, 0.5));
            queue.add(new Task(2000, 5000, Task.LAUNCH, 2000)); // this is just for future integration
            queue.add(new Task(5000, 6000, Task.ROTATE, -0.5));
            queue.add(new Task(5000, 7000, Task.TRANSLATE, -0.5));
        }
        timer.reset();
    }

    @Override
    public void loop() {
        Iterator<Task> iterator = queue.iterator();
        boolean flag = intake != null;
        double time = timer.milliseconds();
        while (iterator.hasNext()) {
            Task task = iterator.next();
            if (time < task.begin) {
                break;
            }
            if (!task.executed) {
                execute(task);
                if (task.type == Task.ROTATE || (task.type == Task.TRANSLATE && task.target >= 0)) { // intake only when launching or moving backwards
                    flag = false;
                }
            } else if (time >= task.end) {
                execute(task);
                iterator.remove();
            }
        }
        if (flag) {
            intake.spin(1);
        }
        drivetrain.run(controls);
        telemetry.update();
    }
    private void execute(Task task) {
        double magnitude = task.executed ? -task.target : task.target;
        switch (task.type) {
            case Task.TRANSLATE: {
                controls.translationYPower += magnitude;
                task.executed = true;
                break;
            }
            case Task.ROTATE: {
                controls.rotationPower += magnitude * color;
                task.executed = true;
                break;
            }
            case Task.LAUNCH: {
                if (!task.executed) {
                    launcher.resetFeeder();
                    launcher.spinToVelocity(task.target);
                    task.executed = true;
                } else {
                    launcher.fire(1);
                }
                break;
            }
            default: {
                break;
            }
        }
    }
}
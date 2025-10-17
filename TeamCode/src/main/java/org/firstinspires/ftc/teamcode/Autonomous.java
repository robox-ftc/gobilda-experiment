package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utils.Drivetrain;
import org.firstinspires.ftc.teamcode.Utils.DrivetrainControls;
import org.firstinspires.ftc.teamcode.Utils.Intake;
import org.firstinspires.ftc.teamcode.Utils.Launcher;
import org.firstinspires.ftc.teamcode.Utils.LauncherControls;
import org.firstinspires.ftc.teamcode.Utils.Task;

import java.util.Iterator;
import java.util.LinkedList;

public class Autonomous {
    public boolean status;
    private long start;
    public LinkedList<Task> queue;
    private DrivetrainControls drivetrainControls;
    private LauncherControls launcherControls;
    private Telemetry telemetry;
    public Autonomous(DrivetrainControls drivetrainControls, LauncherControls launcherControls, Telemetry telemetry) {
        status = true;
        this.drivetrainControls = drivetrainControls;
        this.launcherControls = launcherControls;
        queue = new LinkedList<>();
        start = System.currentTimeMillis();

        // hardcoded tasks
            queue.add(new Task(2000, 3000, 0.5, Task.TRANSLATE));
            queue.add(new Task(3000, 3100, -0.1, Task.ROTATE));
            queue.add(new Task(3100, 5000, 1, Task.LAUNCH));

        this.telemetry = telemetry;
    }

    public void run() {
        if (!queue.isEmpty()) {
            long time = System.currentTimeMillis() - start;
            Task curr = queue.getFirst();
            if (time > curr.end) {
                queue.removeFirst();
            }
            if (time >= curr.begin) {
                execute(curr);
            }
        } else {
            status = false;
        }
    }

    public void execute(Task task) {
        if (task.type == Task.TRANSLATE) {
            drivetrainControls.translationY = task.magnitude;
        } else if (task.type == Task.ROTATE) {
            this.drivetrainControls.rotation = task.magnitude;
            telemetry.addData("currentRotation", drivetrainControls.rotation);
        } else {
            launcherControls.wheelPower = task.magnitude;
        }
    }
}

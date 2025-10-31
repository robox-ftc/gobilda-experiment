package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    public LinkedList<Task> queue;
    private final DrivetrainControls drivetrainControls;
    private final LauncherControls launcherControls;
    private final ElapsedTime timer;
    public Autonomous(DrivetrainControls drivetrainControls, LauncherControls launcherControls) {
        status = true;
        this.drivetrainControls = drivetrainControls;
        this.launcherControls = launcherControls;
        queue = new LinkedList<>();
        timer = new ElapsedTime();
    }
    public void start() { // run upon start (not loop)
        timer.reset();

        // hardcoded tasks
        queue.add(new Task(2000, 2250, 0.5, Task.TRANSLATE));
        queue.add(new Task(3000, 3100, -0.05, Task.ROTATE));
        queue.add(new Task(4000, 8000, 1, Task.LAUNCH));
        queue.add(new Task(8000, 8200, -0.1, Task.ROTATE));
        queue.add(new Task(8250, 8500, 0.5, Task.TRANSLATE));
        queue.add(new Task(8600, 8750, -0.05, Task.ROTATE));
    }

    public void run() {
        if (!queue.isEmpty()) {
            Iterator<Task> iter = queue.iterator();
            Task curr;
            int time = (int) timer.milliseconds();
            while (iter.hasNext()) {
                curr = iter.next();
                if (time >= curr.begin && time <= curr.end) {
                    execute(curr, false);
                } else if (time > curr.end) {
                    execute(curr, true);
                    iter.remove();
                }
            }
        } else {
            status = false;
        }
    }

    public void execute(Task task, boolean stop) {
        if (task.type == Task.TRANSLATE) {
            drivetrainControls.translationY = stop ? 0 : task.magnitude;
        } else if (task.type == Task.ROTATE) {
            drivetrainControls.rotation = stop ? 0 : task.magnitude;
        } else {
            launcherControls.wheelPower = stop ? 0 : task.magnitude;
        }
    }
}

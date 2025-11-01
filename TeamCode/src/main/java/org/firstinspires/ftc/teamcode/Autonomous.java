package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utils.DrivetrainControls;
import org.firstinspires.ftc.teamcode.Utils.LauncherControls;
import org.firstinspires.ftc.teamcode.Utils.Task;

import java.util.Iterator;
import java.util.LinkedList;

public class Autonomous {

    public static final int A = 0;
    public static final int B = 1;
    public static final int X = 2;
    public static final int Y = 3;
    public static final int RED = -1;
    public static final int BLUE = 1;
    public boolean status;
    public LinkedList<Task> queue;
    private final DrivetrainControls drivetrainControls;
    private final LauncherControls launcherControls;
    private final ElapsedTime timer;
    private int flag;
    public Autonomous(DrivetrainControls drivetrainControls, LauncherControls launcherControls) {
        status = true;
        this.drivetrainControls = drivetrainControls;
        this.launcherControls = launcherControls;
        queue = new LinkedList<>();
        timer = new ElapsedTime();
    }
    public void init(int flag) { // run upon init (not loop)
        queue.clear();
        if (flag > B) {
            this.flag = RED;
            flag -= 2;
        } else {
            this.flag = BLUE;
        }
        // hardcoded tasks
        switch (flag) {
            case A:
                queue.add(new Task(2000, 4000, 1, Task.TRANSLATE));
                queue.add(new Task(3250, 3500, 0.25, Task.ROTATE));
            case B:
                queue.add(new Task(2000, 3000, 1, Task.TRANSLATE));
                queue.add(new Task(3000, 3750, 1, Task.ROTATE));
        }
        queue.add(new Task(4000, 10000, 1, Task.LAUNCH));
        queue.add(new Task(10000, 12500, -0.75, Task.TRANSLATE));
        queue.add(new Task(11000, 11500, 0.25, Task.ROTATE));
        queue.add(new Task(13500, 15500, 0.75, Task.TRANSLATE));
        queue.add(new Task(14500, 16000, -0.25, Task.ROTATE));
        queue.add(new Task(16000, 22000, 1, Task.LAUNCH));
        queue.add(new Task(22000, 22500, -0.5, Task.ROTATE));
        queue.add(new Task(22000, 25000, -0.5, Task.TRANSLATE));
        queue.add(new Task(24500, 25000, 1, Task.ROTATE));
    }
    public void start() {
        timer.reset();
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
            drivetrainControls.rotation = stop ? 0 : task.magnitude * flag;
        } else {
            launcherControls.wheelPower = stop ? 0 : task.magnitude;
        }
    }
}

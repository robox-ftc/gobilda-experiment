package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utils.Drivetrain;
import org.firstinspires.ftc.teamcode.Utils.GamePadReadings;
import org.firstinspires.ftc.teamcode.Utils.Intake;
import org.firstinspires.ftc.teamcode.Utils.Launcher;
import org.firstinspires.ftc.teamcode.Utils.Task;

import java.util.Iterator;
import java.util.LinkedList;

@Autonomous(name = "Auto Mode", group = "Robot")
public class AutonomousOpMode extends OpMode {

    public static final int TRANSLATION_FACTOR = 1;
    public static final int ROTATION_FACTOR = 100;
    public static final double LAUNCHER_FACTOR = 0.01;
    public static final int RED = -1;
    public static final int BLUE = 1;
    public static final int A = 0;
    public static final int B = 1;
    public static final int X = 2;
    public static final int Y = 3;
    private Launcher launcher = null;
    private Drivetrain drivetrain = null;
    private Intake intake = null;
    private int mode;
    private int color;
    public LinkedList<Task> queue;
    private ElapsedTime timer;
    private GoBildaPinpointDriver odo;

    public void init() {
        launcher = new Launcher(hardwareMap, telemetry);
        drivetrain = new Drivetrain(hardwareMap, telemetry, true);
        intake = new Intake(hardwareMap, telemetry);
        queue = new LinkedList<>();
        timer = new ElapsedTime();
    }
    public void init_loop() {
        queue.clear();
        if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
            mode = A;
            color = BLUE;
        } else if (gamepad1.bWasPressed() || gamepad2.bWasPressed()) {
            mode = B;
            color = BLUE;
        } else if (gamepad1.xWasPressed() || gamepad2.xWasPressed()) {
            mode = A;
            color = RED;
        } else if (gamepad1.yWasPressed() || gamepad2.yWasPressed()) {
            mode = B;
            color = RED;
        }
        switch (mode) {
            case A:
                queue.add(new Task(2000, 4000, Task.TRANSLATE, 75));
                queue.add(new Task(3250, 3500, Task.ROTATE, 45));
            case B:
                queue.add(new Task(2000, 3000, Task.TRANSLATE, 50));
                queue.add(new Task(3000, 3700, Task.ROTATE, 180));
        }
        queue.add(new Task(4000, 10000, Task.LAUNCH, 75));
        queue.add(new Task(10000, 12500, Task.TRANSLATE, -50));
        queue.add(new Task(11000, 11500, Task.ROTATE, 45));
        queue.add(new Task(13500, 15500, Task.TRANSLATE, 50));
        queue.add(new Task(14500, 16000, Task.ROTATE, -45));
        queue.add(new Task(16000, 22000, Task.LAUNCH, 75));
        queue.add(new Task(22000, 22500, Task.ROTATE, -90));
        queue.add(new Task(22000, 25000, Task.TRANSLATE, 75));
        queue.add(new Task(24500, 25000, Task.ROTATE, 45));
    }

    public void start() {
        timer.reset();
    }
    public void loop() {
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
        }
        intake.run(true);
        drivetrain.run(true);
    }
    public void execute(Task task, boolean stop) {
        int target = stop ? 0 : task.target;
        if (task.type == Task.TRANSLATE) {
            target *= TRANSLATION_FACTOR;
            drivetrain.setTargets(new int[]{target, target, target, target});
        } else if (task.type == Task.ROTATE) {
            target *= color;
            drivetrain.setTargets(new int[]{target, target, target, target});
        } else {
            launcher.targetSpeed = target;
            if (!stop) {
                launcher.launch(true);
            }
        }
    }
}

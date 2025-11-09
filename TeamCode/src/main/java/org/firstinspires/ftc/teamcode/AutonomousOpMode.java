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
        queue.add(new Task(20000, 22000 + (mode * 1000), Task.TRANSLATE, 1));
        if (!queue.isEmpty()) {
            switch (mode) {
                case A:
                    queue.add(new Task(4000, 6000, Task.TRANSLATE, 1));
                case B:
                    queue.add(new Task(4000, 10000, Task.TRANSLATE, 1));
            }
        }
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
        double target = stop ? 0 : task.target;
        if (task.type == Task.TRANSLATE) {
            drivetrain.setPowers(new double[]{target, -target, -target, target});
        } else if (task.type == Task.ROTATE) {
            target *= color;
            drivetrain.setPowers(new double[]{target, target, target, target});
        } else {
            launcher.targetSpeed = target;
            if (!stop) {
                launcher.launch(true);
            }
        }
    }
}

//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Old.GoBildaPinpointDriver;
//import org.firstinspires.ftc.teamcode.Utils.Drivetrain;
//import org.firstinspires.ftc.teamcode.Utils.DrivetrainControls;
//import org.firstinspires.ftc.teamcode.Utils.Launcher;
//import org.firstinspires.ftc.teamcode.Utils.LauncherControls;
//import org.firstinspires.ftc.teamcode.Utils.Task;
//import org.opencv.core.Mat;
//
//import java.util.Iterator;
//import java.util.LinkedList;
//
//public class Autonomous {
//
//    public static final int A = 0;
//    public static final int B = 1;
//    public static final int X = 2;
//    public static final int Y = 3;
//    public static final int RED = 1;
//    public static final int BLUE = -1;
//    public boolean status;
//    public LinkedList<Task> queue;
//    private final Drivetrain drivetrain;
//    private final Launcher launcher;
//    private final ElapsedTime timer;
//    private GoBildaPinpointDriver odo;
//    private int flag;
//    public Autonomous(Drivetrain drivetrain, Launcher launcher, HardwareMap map) {
//        status = true;
//        this.odo = map.get(GoBildaPinpointDriver.class, "odo");
//        this.drivetrain = drivetrain;
//        this.launcher = launcher;
//        queue = new LinkedList<>();
//        timer = new ElapsedTime();
//    }
//    public void init(int flag) { // run upon init (not loop)
//        queue.clear();
//        if (flag > B) {
//            this.flag = RED;
//            flag -= 2;
//        } else {
//            this.flag = BLUE;
//        }
//        // hardcoded tasks
//        switch (flag) {
//            case A:
//                queue.add(new Task(2000, 4000, Task.TRANSLATE, 75));
//                queue.add(new Task(3250, 3500, Task.ROTATE, 45));
//            case B:
//                queue.add(new Task(2000, 3000, Task.TRANSLATE, 50));
//                queue.add(new Task(3000, 3700, Task.ROTATE, 180));
//        }
//        queue.add(new Task(4000, 10000, Task.LAUNCH, 100));
//        queue.add(new Task(10000, 12500, Task.TRANSLATE, -50));
//        queue.add(new Task(11000, 11500, Task.ROTATE, 45));
//        queue.add(new Task(13500, 15500, Task.TRANSLATE, 50));
//        queue.add(new Task(14500, 16000, Task.ROTATE, -45));
//        queue.add(new Task(16000, 22000, Task.LAUNCH, 100));
//        queue.add(new Task(22000, 22500, Task.ROTATE, -90));
//        queue.add(new Task(22000, 25000, Task.TRANSLATE, 75));
//        queue.add(new Task(24500, 25000, Task.ROTATE, 45));
//    }
//    public void start() {
//        timer.reset();
//    }
//
//    public void run() {
//        if (!queue.isEmpty()) {
//            Iterator<Task> iter = queue.iterator();
//            Task curr;
//            int time = (int) timer.milliseconds();
//            while (iter.hasNext()) {
//                curr = iter.next();
//                if (time >= curr.begin && time <= curr.end) {
//                    execute(curr, false);
//                } else if (time > curr.end) {
//                    execute(curr, true);
//                    iter.remove();
//                }
//            }
//        } else {
//            status = false;
//        }
//    }
//
//    public void execute(Task task, boolean stop) {
//        int target = task.target;
//        if (task.type == Task.TRANSLATE) {
//            drivetrain.setTargets(new int[]{target, target, target, target});
//        } else if (task.type == Task.ROTATE) {
//            target *= flag;
//            drivetrain.setTargets(new int[]{target, target, target, target});
//        } else {
//            launcher. = stop ? 0 : task.magnitude;
//        }
//    }
//}

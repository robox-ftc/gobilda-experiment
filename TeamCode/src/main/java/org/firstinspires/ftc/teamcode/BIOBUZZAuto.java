package org.firstinspires.ftc.teamcode;

//import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.devices.Drivetrain;
import org.firstinspires.ftc.teamcode.devices.DrivetrainControls;
import org.firstinspires.ftc.teamcode.devices.Intake;
import org.firstinspires.ftc.teamcode.devices.Launcher;
import org.firstinspires.ftc.teamcode.devices.Ramp;
import org.firstinspires.ftc.teamcode.devices.Task;

import java.util.*;
@Autonomous(name = "HiveKnightAuto", group = "Robot")
public class BIOBUZZAuto extends OpMode {
    private static final double TOLERANCE = 1; // Arbitrary
    private static final double RTOLERANCE = 5; // Arbitrary

    // Tunable P
    private final double P = 1.0 / 20.0;
    // Rotation Factor
    private final double R = 2 * 12.0 / 360.0;
    private boolean launcherStarted;

//    private GoBildaPinpointDriver pinpoint;
    private Drivetrain drivetrain;
    private Intake intake;
    private Launcher launcher;
    private Ramp ramp;
    private DrivetrainControls controls;
    private boolean HiveSide;
    private boolean SideSelected;
    private boolean OdometrySelected;
    private boolean TaskStarted;
//    private boolean PinpointUsed;
    private Queue<Task> queue;
    private ElapsedTime timer;
    ElapsedTime launcherTimer = new ElapsedTime();
    ElapsedTime SpinUpTimer = new ElapsedTime();
    private enum AutoMode{
        Leave,
        Park,
        Launch,
        LaunchAndPark,
        Intake,
    }
    AutoMode autoMode = null;
    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap, telemetry, false);
        controls = new DrivetrainControls();
        ramp = new Ramp(hardwareMap, telemetry);
        telemetry.addLine("Drivetrain initialized");
//        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
//        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        pinpoint.setOffsets(0, 7.5, DistanceUnit.INCH); //Likely needs to be changed
//        pinpoint.resetPosAndIMU();
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
        HiveSide = true;
        SideSelected = false;
        OdometrySelected = false;
        launcherStarted = false;
        TaskStarted = false;
        queue = new LinkedList<>();
        timer = new ElapsedTime();
        telemetry.update();
    }
    @Override
    public void init_loop() {
        if (!SideSelected) {
            if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
                HiveSide = true;
                SideSelected = true;
            }
            if ((gamepad1.bWasPressed() || gamepad2.bWasPressed()) && !SideSelected) {
                // Probably unnecessary as it only prevents two sides from being selected if two buttons are pressed simultaneously
                HiveSide = false;
                SideSelected = true;
            }
        }
            else if (SideSelected) {
                if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
                    autoMode = AutoMode.Leave;
                }
                else if (gamepad1.bWasPressed() || gamepad2.bWasPressed()) {
                    autoMode = AutoMode.Park;
                }
                else if (gamepad1.xWasPressed() || gamepad2.xWasPressed()) {
                    autoMode = AutoMode.Launch;
                }
                else if (gamepad1.yWasPressed() || gamepad2.yWasPressed()) {
                    autoMode = AutoMode.LaunchAndPark;
                }
                else if (gamepad1.leftBumperWasPressed() || gamepad2.leftBumperWasPressed()) {
                    autoMode = AutoMode.Intake;
                }
                else if (gamepad1.rightBumperWasPressed() || gamepad2.rightBumperWasPressed()) {
                    SideSelected = false;
                    autoMode = null;
                }
            }
        if (SideSelected) {
            telemetry.addLine(HiveSide ? "Hive Side is Selected" : "Wall Side is Selected");
        }
        else {
            telemetry.addLine("Select Side; A=Hive, B=Wall");
        }

        if (autoMode != null) {
            telemetry.addLine(autoMode + " is Selected");
            telemetry.addLine("RB to reset");
        }
        else {
            telemetry.addLine("Select Auto Mode; A=Leave, B=Park, X=Launch, Y=Launch And Park");
        }
        telemetry.update();
    }
    @Override
    public void start() {
        if (autoMode == null){
            autoMode = AutoMode.Leave;
        }

        switch (autoMode) {
            case Leave:
                    queue.add(new Task(0, 1000, Task.Type.TRANSLATE, 0, 0.25)); //Time and power settings
                break;

            case Park:
                if (HiveSide) {
                    queue.add(new Task(0, 3000, Task.Type.TRANSLATE, -0.14, 0.25));
                } else {
                    queue.add(new Task(0, 3000, Task.Type.TRANSLATE, 0.25, 0.14));
                }
                break;

            case Launch:
                if (HiveSide) {
                    queue.add(new Task(0, 6000, Task.Type.LAUNCH, 0.8, 3000));
                } else {
                    queue.add(new Task(0, 3000, Task.Type.TRANSLATE, 0.25, 0.14));
                    queue.add(new Task(0, 810, Task.Type.ROTATE, -0.5, 0));
                    queue.add(new Task(0, 6000, Task.Type.LAUNCH, 0.8, 3000));
                }
                break;

            case LaunchAndPark:
                if (HiveSide) {
                    queue.add(new Task(0, 6000, Task.Type.LAUNCH, 0.8, 3000)); //Time, Power, and spin up time
                    queue.add(new Task(0, 3000, Task.Type.TRANSLATE, -0.14, 0.25));
                } else {
                    queue.add(new Task(0, 3000, Task.Type.TRANSLATE, 0.25, 0.14));
                    queue.add(new Task(0, 810, Task.Type.ROTATE, -0.5, 0));
                    queue.add(new Task(0, 6000, Task.Type.LAUNCH, 0.8, 3000));
                    queue.add(new Task(0, 3000, Task.Type.TRANSLATE, 0.25, 0.14));
                }
                break;
            case Intake:
                if (HiveSide) {
                    queue.add(new Task(0, 6000, Task.Type.LAUNCH, 0.8, 3000)); //Time, Power, and spin up time
                    queue.add(new Task(0, 810, Task.Type.ROTATE, 0.5, 0));
                    queue.add(new Task(0, 2000, Task.Type.TRANSLATE, 0, -0.5));
                    queue.add(new Task(0, 2000, Task.Type.TRANSLATE, 0, 0.5));
                    queue.add(new Task(0, 810, Task.Type.ROTATE, -0.5, 0));
                    queue.add(new Task(0, 6000, Task.Type.LAUNCH, 0.8, 3000));
                    queue.add(new Task(0, 3000, Task.Type.TRANSLATE, -0.14, 0.25));
                } else {
                    queue.add(new Task(0, 3000, Task.Type.TRANSLATE, 0.25, 0.14));
                    queue.add(new Task(0, 810, Task.Type.ROTATE, -0.5, 0));
                    queue.add(new Task(0, 6000, Task.Type.LAUNCH, 0.8, 3000));
                    queue.add(new Task(0, 810, Task.Type.ROTATE, 0.5, 0));
                    queue.add(new Task(0, 2000, Task.Type.TRANSLATE, 0, -0.5));
                    queue.add(new Task(0, 2000, Task.Type.TRANSLATE, 0, 0.5));
                    queue.add(new Task(0, 810, Task.Type.ROTATE, -0.5, 0));
                    queue.add(new Task(0, 6000, Task.Type.LAUNCH, 0.8, 3000));
                    queue.add(new Task(0, 3000, Task.Type.TRANSLATE, -0.14, 0.25));
                }
        }
        timer.reset();
    }
    @Override
    public void loop() {
        controls.translationXPower = 0;
        controls.translationYPower = 0;
        controls.rotationPower = 0;
        if (!queue.isEmpty()) {
            Task task = queue.peek();
            if (!TaskStarted) {
                timer.reset();
                TaskStarted = true;
            }
            switch (task.type) {
                case TRANSLATE:
                    double x = task.target;
                    double y = task.target2;
                    if (timer.milliseconds() > task.end) {
                        controls.translationXPower = 0;
                        controls.translationYPower = 0;
                        if (intake != null) {
                            intake.spin(0);
                        }
                        queue.remove();
                        TaskStarted = false;
                    } else {
                        controls.translationXPower = pClamp(x);
                        controls.translationYPower = pClamp(y);
                        if (intake != null) {
                            intake.spin(1);
                        }
                    }
                    break;

                    case ROTATE:
                        double error = task.target;
                        if (error > 180) error -= 360;
                        if (error < -180) error += 360;
                        if (timer.milliseconds() > task.end) {
                            controls.rotationPower = 0;
                            queue.remove();
                            TaskStarted = false;
                        } else {
                            controls.rotationPower = pClamp(task.target);
                        }
                        break;

                case LAUNCH:
                    if (intake != null && launcher != null) {

                        if (!launcherStarted) {
                            launcherTimer.reset();
                            SpinUpTimer.reset();
                            launcherStarted = true;
                        }

                        if (SpinUpTimer.milliseconds() < task.target2) {
                            launcher.spin(task.target);
                        }

                        else if (launcherTimer.milliseconds() < task.end) {
                            launcher.spin(task.target);
                            ramp.spin(1);
                        }
                        else {
                            launcher.spin(0);
                            ramp.spin(0);

                            queue.remove();
                            launcherStarted = false;
                            TaskStarted = false;
                        }

                    } else {
                        queue.remove();
                        launcherStarted = false;
                        TaskStarted = false;
                    }
                    break;

                default:
                    controls.translationXPower = 0;
                    controls.translationYPower = 0;
                    controls.rotationPower = 0;
                    TaskStarted = false;
                    launcherStarted = false;
                    queue.remove();
                    break;
            }
        } else {
            if (intake != null) {
                intake.spin(0);
            }
        }
        if (drivetrain != null) drivetrain.run(controls);
//if (PinpointUsed) {
    //telemetry.addData("Current X", x);
    //telemetry.addData("Current Y", y);
    //telemetry.addData("Current Heading", heading);
    //telemetry.addData("Remaining Tasks", queue.size());
//}
        telemetry.update();
    }
    private double pClamp(double error) {
        return Math.min(Math.max(error, -1), 1);
    }
}
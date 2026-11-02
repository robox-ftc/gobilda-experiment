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
import org.firstinspires.ftc.teamcode.devices.Turret;

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
    private Turret turret;
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
    }
    AutoMode autoMode = AutoMode.Leave;
    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap, telemetry, false);
        controls = new DrivetrainControls();
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
                telemetry.addLine("Hive Side is Selected");
                HiveSide = true;
                SideSelected = true;
            }
            if ((gamepad1.bWasPressed() || gamepad2.bWasPressed()) && !SideSelected) {
                // Probably unnecessary as it only prevents two sides from being selected if two buttons are pressed simultaneously
                telemetry.addLine("Wall Side is Selected");
                HiveSide = false;
                SideSelected = true;
            }
        }
        else {
            if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
                telemetry.addLine("Leave is Selected");
                autoMode = AutoMode.Leave;
            }
            if (gamepad1.bWasPressed() || gamepad2.bWasPressed()) {
                telemetry.addLine("Park is Selected");
                autoMode = AutoMode.Park;
            }
            if (gamepad1.xWasPressed() || gamepad2.xWasPressed()) {
                telemetry.addLine("Launch is Selected");
                autoMode = AutoMode.Launch;
            }
            if (gamepad1.yWasPressed() || gamepad2.yWasPressed()) {
                telemetry.addLine("LaunchAndPark is Selected");
                autoMode = AutoMode.LaunchAndPark;
            }
            if (gamepad1.rightBumperWasPressed() || gamepad2.rightBumperWasPressed()) {
                SideSelected = false;
            }
        }
//        if (SideSelected && !OdometrySelected) {
//            if (gamepad1.leftBumperWasPressed() || gamepad2.leftBumperWasPressed()) {
//                telemetry.addLine("Pinpoint is enabled");
//                OdometrySelected = true;
//                PinpointUsed = true;
//            }
//            if (gamepad1.rightBumperWasPressed() || gamepad2.rightBumperWasPressed()) {
//                telemetry.addLine("Pinpoint is disabled");
//                OdometrySelected = true;
//                PinpointUsed = false;
//            }
//        }
        telemetry.update();
    }
    @Override
    public void start() {
        if (!SideSelected){
            autoMode = AutoMode.Leave;
        }

        switch (autoMode) {
            case Leave:
                    queue.add(new Task(0, 500, Task.Type.TRANSLATE, 0, 0.5)); //Time and power settings
                break;

            case Park:
                if (HiveSide) {
                    queue.add(new Task(0, 1000, Task.Type.TRANSLATE, 0.28, 0.5));
                } else {
                    queue.add(new Task(0, 1000, Task.Type.TRANSLATE, 0.5, 0.28));
                }
                break;

            case Launch:
                if (HiveSide) {
                    queue.add(new Task(0, 10000, Task.Type.LAUNCH, 0, 5));
                } else {
                    queue.add(new Task(0, 1500, Task.Type.TRANSLATE, 0, 1));
                    queue.add(new Task(0, 1000, Task.Type.ROTATE, -90, 0));
                    queue.add(new Task(0, 10000, Task.Type.LAUNCH, 0, 5));
                }
                break;

            case LaunchAndPark:
                if (HiveSide) {
                    queue.add(new Task(0, 7000, Task.Type.LAUNCH, 0, 3)); //Power and spin-up time
                    queue.add(new Task(0, 3000, Task.Type.TRANSLATE, 0.9, 0.75));
                } else {
                    queue.add(new Task(0, 1500, Task.Type.TRANSLATE, 0, 1));
                    queue.add(new Task(0, 1000, Task.Type.ROTATE, -0.75, 0)); //Motor power
                    queue.add(new Task(0, 10000, Task.Type.LAUNCH, 0, 5));
                    queue.add(new Task(0, 4000, Task.Type.TRANSLATE, 0.9, 0.75));
                }
                break;

        }
        timer.reset();
    }
    @Override
    public void loop() {
        controls.translationXPower = 0;
        controls.translationYPower = 0;
        controls.rotationPower = 0;
//if (PinpointUsed) {
//        pinpoint.update();
//        double x = pinpoint.getPosX(DistanceUnit.INCH);
//        double y = pinpoint.getPosY(DistanceUnit.INCH);
//        double heading = pinpoint.getHeading(AngleUnit.DEGREES);
//}
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
                        intake.spin(1);
                        queue.remove();
                        TaskStarted = false;
                    } else {
                        controls.translationXPower = pClamp(x);
                        controls.translationYPower = pClamp(y);
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
                            controls.rotationPower = pClamp(error * R);
                        }
                        break;

                case LAUNCH:
                    if (intake != null && launcher != null && !launcherStarted) {
                        launcherTimer.reset();
                        SpinUpTimer.reset();
                        launcherStarted = true;
                        TaskStarted = false;

                        if (SpinUpTimer.seconds() < task.target2) {
                            launcher.spin(task.target);
                        }
                        else if (launcherTimer.seconds() < task.end) {
                            launcher.spin(task.target);
                            ramp.spin(1);
                        }
                    }
                    else {
                        queue.remove();
                        TaskStarted = false;
                        launcherStarted = false;
                    }
                    break;

                default:
                    controls.translationXPower = 0;
                    controls.translationYPower = 0;
                    controls.rotationPower = 0;
                    queue.remove();
                    break;
            }
        } else {
            if (intake != null) {
                intake.spin(0);
            }
            if (turret != null) {
                turret.resetTurret();
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
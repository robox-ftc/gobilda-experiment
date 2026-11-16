package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
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
import org.firstinspires.ftc.teamcode.devices.Task2;

import java.util.*;
@Autonomous(name = "HiveKnightAuto-Odometry", group = "Robot")
public class BIOBUZZAutoOdo extends OpMode {
    private static final double TRANSLATION_TOLERANCE = 3;
    private static final double ROTATION_TOLERANCE = 1.0;
    private static final double driveP = 0.08 ; // New: 0.1
    //Original: 0.08. Controls how "careful" the robot is.
    // Unrelated to tolerance, but basically high value-fast, inaccurate and small value=slow, accurate
    private static final double rotationP = 0.08; //New: 0.04
    //Original: 0.08. Same as driveP (I think -Jasper)
    private static final double MAX_POWER = 1;
    private static final double MIN_POWER = 0.01;
    private boolean launcherStarted;

    private GoBildaPinpointDriver pinpoint;
    private Drivetrain drivetrain;
    private Intake intake;
    private Launcher launcher;
    private Ramp ramp;
    private DrivetrainControls controls;
    private boolean HiveSide;
    private boolean SideSelected;
    private boolean TaskStarted;
    private Queue<Task2> queue;
    private ElapsedTime timer;
    private ElapsedTime rotationSettleTimer = new ElapsedTime();
    private boolean rotationSettling = false;
    private static final double SETTLE_TIME = 200;
    ElapsedTime launcherTimer = new ElapsedTime();
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
        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            pinpoint.setEncoderDirections(
                    GoBildaPinpointDriver.EncoderDirection.FORWARD,
                    GoBildaPinpointDriver.EncoderDirection.FORWARD);
            pinpoint.setOffsets(0, 7.5, DistanceUnit.INCH); //Needs to be changed
            telemetry.addLine("Odometry initialized");
            pinpoint.resetPosAndIMU();
        } catch (Exception e) {
            telemetry.addLine("Odometry initialization failed");
            //Not useful, as if the odometry is broken the auto and will always throw an exception.
            //Time based BIOBUZZ auto can be used instead, when it has been tested
        }
        try {
            launcher = new Launcher(hardwareMap, telemetry);
            telemetry.addLine("Launcher initialized");
        } catch (Exception e) {
            launcher = null;
            telemetry.addLine("Launcher initialization failed");
        }
        try {
            intake = new Intake(hardwareMap, telemetry);
        } catch (Exception e) {
            intake = null;
            telemetry.addLine("Intake initialized");
        }
        HiveSide = true;
        SideSelected = false;
        launcherStarted = false;
        TaskStarted = false;
        queue = new LinkedList<>();
        timer = new ElapsedTime();
        telemetry.update();

    }
    @Override
    public void init_loop() {
        pinpoint.update();
        double x = pinpoint.getPosX(DistanceUnit.INCH);
        double y = pinpoint.getPosY(DistanceUnit.INCH);
        double heading = pinpoint.getHeading(AngleUnit.DEGREES);
        if (!SideSelected) {
            if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
                HiveSide = true;
                SideSelected = true;
            }
            if ((gamepad1.bWasPressed() || gamepad2.bWasPressed()) && !SideSelected){
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
            telemetry.addLine("Select Auto Mode; A=Leave, B=Park, X=Launch, Y=Launch And Park, LB=Intake");
        }
        telemetry.addData("Current X", x);
        telemetry.addData("Current Y", y);
        telemetry.addData("Current Heading", heading);
        telemetry.update();
    }
    @Override
    public void start() {
        if (autoMode == null){
            autoMode = AutoMode.Park;
        }
        //Format for tasks:
        //Start time not used for any. End time is always maximum time the task can take
        //Translate: Uses coordinates relative to field, odometry pods at (0, 0)
        //Rotate: Target heading. Not how much to rotate by, but what heading relative to field. Y parameter unused
        //Launch: How long to spin up the launcher (2 seconds should be good) and total launch time.
        //The ramp spins and causes a launch when the spin up time is completed.
        switch (autoMode) {
            case Leave:
                queue.add(new Task2(3000, Task2.Type.TRANSLATE, 0, 24)); //Time and distance settings
                break;

            case Park:
                if (HiveSide) {
                    queue.add(new Task2(2000, Task2.Type.TRANSLATE, -12, 12));
                    queue.add(new Task2(4000, Task2.Type.TRANSLATE, -46, 96));
                } else {
                    queue.add(new Task2(2000, Task2.Type.TRANSLATE, 0, 6));
                }
                break;

            case Launch:
                if (HiveSide) {
                    queue.add(new Task2(5000, Task2.Type.LAUNCH, 0.9, 2000));
                } else {
                    queue.add(new Task2(2000, Task2.Type.TRANSLATE, 0, 12));
                    queue.add(new Task2(2000, Task2.Type.TRANSLATE, 72, 12));
                    queue.add(new Task2(2000, Task2.Type.TRANSLATE, 72, 36));
                    queue.add(new Task2(3000, Task2.Type.ROTATE, -90));
                    queue.add(new Task2(2000, Task2.Type.TRANSLATE, 100, 36));
                    queue.add(new Task2(5000, Task2.Type.LAUNCH, 0.9, 2000));
                }
                break;

            case LaunchAndPark:
                if (HiveSide) {
                    queue.add(new Task2(5000, Task2.Type.LAUNCH, 0.9, 2000));
                    queue.add(new Task2(2000, Task2.Type.TRANSLATE, -12, 12));
                    queue.add(new Task2(4000, Task2.Type.TRANSLATE, -46, 96));
                } else {
                    queue.add(new Task2(2000, Task2.Type.TRANSLATE, 0, 12));
                    queue.add(new Task2(2000, Task2.Type.TRANSLATE, 72, 12));
                    queue.add(new Task2(2000, Task2.Type.TRANSLATE, 72, 36));
                    queue.add(new Task2(3000, Task2.Type.ROTATE, -90));
                    queue.add(new Task2(2000, Task2.Type.TRANSLATE, 100, 36));
                    queue.add(new Task2(5000, Task2.Type.LAUNCH, 0.9, 2000));
                    queue.add(new Task2(2000, Task2.Type.TRANSLATE, 88, 24));
                    queue.add(new Task2(4000, Task2.Type.TRANSLATE, 0, 6));
                }
                break;
            case Intake:
                if (HiveSide) {
                    queue.add(new Task2(5000, Task2.Type.LAUNCH, 0.9, 2000));
                    queue.add(new Task2(4000, Task2.Type.TRANSLATE, 0, 6));
                    queue.add(new Task2(3000, Task2.Type.ROTATE, 90));
                    queue.add(new Task2(2000, Task2.Type.TRANSLATE, -48, 6));
                    queue.add(new Task2(4000, Task2.Type.TRANSLATE, 0, 6));
                    queue.add(new Task2(3000, Task2.Type.ROTATE, 0));
                    queue.add(new Task2(5000, Task2.Type.LAUNCH, 0.9, 2000));
                    queue.add(new Task2(2000, Task2.Type.TRANSLATE, -12, 12));
                    queue.add(new Task2(4000, Task2.Type.TRANSLATE, -46, 96));
                } else {
                    queue.add(new Task2(2000, Task2.Type.TRANSLATE, 0, 12));
                    queue.add(new Task2(2000, Task2.Type.TRANSLATE, 72, 12));
                    queue.add(new Task2(2000, Task2.Type.TRANSLATE, 72, 36));
                    queue.add(new Task2(3000, Task2.Type.ROTATE, -90));
                    queue.add(new Task2(2000, Task2.Type.TRANSLATE, 100, 36));
                    queue.add(new Task2(5000, Task2.Type.LAUNCH, 0.9, 2000));
                    queue.add(new Task2(2000, Task2.Type.TRANSLATE, 94, 36));
                    queue.add(new Task2(3000, Task2.Type.ROTATE, 0));
                    queue.add(new Task2(2000, Task2.Type.TRANSLATE, 94, 0));
                    queue.add(new Task2(2000, Task2.Type.TRANSLATE, 94, 36));
                    queue.add(new Task2(3000, Task2.Type.ROTATE, -90));
                    queue.add(new Task2(5000, Task2.Type.LAUNCH, 0.9, 2000));
                    queue.add(new Task2(2000, Task2.Type.TRANSLATE, 88, 24));
                    queue.add(new Task2(4000, Task2.Type.TRANSLATE, 0, 6));
                }
        }
        timer.reset();
    }
    @Override
    public void loop() {
        pinpoint.update();
        double x = pinpoint.getPosX(DistanceUnit.INCH);
        double y = pinpoint.getPosY(DistanceUnit.INCH);
        double heading = pinpoint.getHeading(AngleUnit.DEGREES);
        controls.translationXPower = 0;
        controls.translationYPower = 0;
        controls.rotationPower = 0;

        if (!queue.isEmpty()) {
            Task2 task2 = queue.peek();
            if (!TaskStarted) {
                timer.reset();
                TaskStarted = true;
            }
            switch (task2.type) {
                case TRANSLATE:
                    double xTarget = task2.FieldX; // Field-right, relative to the start pose
                    double yTarget = task2.FieldY; // Field-forward, relative to the start pose

                    // Pinpoint axes are X-forward and Y-left; auto targets use X-right and Y-forward.
                    double xError = yTarget - x;
                    double yError = -xTarget - y;

                    double distance = Math.hypot(xError, yError);

                    if (distance < TRANSLATION_TOLERANCE || timer.milliseconds() > task2.end) {
                        controls.translationXPower = 0;
                        controls.translationYPower = 0;
                        controls.rotationPower = 0;

                        if (intake != null) {
                            intake.spin(0);
                        }
                        queue.remove();
                        TaskStarted = false;

                    } else {
                        double headingRad = Math.toRadians(heading); // Pinpoint heading is counterclockwise-positive
                        double sinHeading = Math.sin(headingRad);
                        double cosHeading = Math.cos(headingRad);
                        // Convert Pinpoint-frame error (forward, left) to robot controls (right, forward).
                        double robotStrafeError = xError * sinHeading - yError * cosHeading;
                        double robotForwardError = xError * cosHeading + yError * sinHeading;

                        telemetry.addData("robotStrafeError", robotStrafeError);
                        telemetry.addData("robotForwardError", robotForwardError);

                        controls.translationXPower = translationPower(robotStrafeError);
                        controls.translationYPower = translationPower(robotForwardError);
                        if (intake != null) {
                            intake.spin(1);
                        }
                    }
                    telemetry.addData("X Target", xTarget); //Unnecessary information, only used for debugging
                    telemetry.addData("Y Target", yTarget);
                    telemetry.addData("Heading", heading);
                    telemetry.addData("X Error", xError);
                    telemetry.addData("Y Error", yError);
                    telemetry.addData("Distance Remaining", distance);
                    telemetry.addData("X Power", controls.translationXPower);
                    telemetry.addData("Y Power", controls.translationYPower);
                    telemetry.addData("Task Started", TaskStarted);
                    break;

                case ROTATE:
                    double targetHeading = task2.Heading;

                    double error = targetHeading - heading;

                    while (error > 180) error -= 360;
                    while (error < -180) error += 360;

                    if (Math.abs(error) < ROTATION_TOLERANCE) {

                        controls.rotationPower = 0;

                        if (!rotationSettling) { //Possibly not necessary
                            rotationSettleTimer.reset();
                            rotationSettling = true;
                        }
                        if (rotationSettleTimer.milliseconds() >= SETTLE_TIME) {
                            queue.remove();
                            TaskStarted = false;
                            rotationSettling = false;
                       }
                    } else {
                        rotationSettling = false;
                        // Positive drivetrain rotation is clockwise; Pinpoint heading is counterclockwise-positive.
                        controls.rotationPower = -rotationPower(error);
                    }

                    telemetry.addData("Heading Error", error);
                    telemetry.addData("Settling", rotationSettling);

                    break;

                case LAUNCH:
                    if (intake != null && launcher != null) {

                        if (!launcherStarted) {
                            launcherTimer.reset();
                            launcherStarted = true;
                        }

                        if (launcherTimer.milliseconds() < task2.FieldY) {
                            launcher.spin(task2.FieldX);
                        }

                        else if (launcherTimer.milliseconds() < task2.end) {
                            launcher.spin(task2.LauncherPower);
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
            if (intake != null) intake.spin(0);
            if (launcher != null) launcher.spin(0);
            if (ramp != null) ramp.spin(0);

            if (drivetrain != null) {
                controls.translationXPower = 0;
                controls.translationYPower = 0;
                controls.rotationPower = 0;
            }
        }

        if (drivetrain != null) drivetrain.run(controls);
        telemetry.addData("Current X", x);
        telemetry.addData("Current Y", y);
        telemetry.addData("Current Heading", heading);
        telemetry.addData("Remaining Tasks", queue.size());
        telemetry.update();
    }
    private double translationPower(double error) {
        double power = error * driveP;

        power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));

        if (Math.abs(power) < MIN_POWER && error != 0) {
            power = Math.signum(power) * MIN_POWER;
        }
        return power;
    }
    private double rotationPower(double error) {

        if (Math.abs(error) < ROTATION_TOLERANCE) {
            return 0;
        }
        double power = error * rotationP;

        power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));

        if (Math.abs(power) < MIN_POWER) {
            power = Math.signum(power) * MIN_POWER;
        }

        return power;
    }

}
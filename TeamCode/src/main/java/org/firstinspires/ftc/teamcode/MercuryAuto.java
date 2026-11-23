package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.devices.Drivetrain;
import org.firstinspires.ftc.teamcode.devices.DrivetrainControls;
import org.firstinspires.ftc.teamcode.devices.Intake;
import org.firstinspires.ftc.teamcode.devices.Launcher;
import org.firstinspires.ftc.teamcode.devices.PinpointSetup;
import org.firstinspires.ftc.teamcode.devices.Ramp;
import org.firstinspires.ftc.teamcode.devices.Task2;

import java.util.*;
@Autonomous(name = "HiveKnightAuto-Mercury", group = "Robot")
public class MercuryAuto extends OpMode {
    private static final double MAX_POWER = 1;
    private static final double MIN_POWER = 0.05;
    private static final double driveP = 0.08;
    private static final double rotP = 0.08;
    private static final double DriveTolerance = 3;
    private static final double RotationTolerance = 3;
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
    ElapsedTime launcherTimer = new ElapsedTime();

    private enum AutoMode {
        Leave,
        Park,
        Launch,
        LaunchAndPark,
        Intake,
    }
    MercuryAuto.AutoMode autoMode = null;

    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap, telemetry, false);
        controls = new DrivetrainControls();
        ramp = new Ramp(hardwareMap, telemetry);
        telemetry.addLine("Drivetrain initialized");
        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
            pinpoint.setOffsets(4, 5, DistanceUnit.INCH); //May need to be changed
            telemetry.addLine("Odometry initialized");
            pinpoint.resetPosAndIMU();
        } catch (Exception e) {
            telemetry.addLine("Odometry initialization failed, use a different auto");
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
            telemetry.addLine("Intake initialized");
        } catch (Exception e) {
            intake = null;
            telemetry.addLine("Intake failed to initialized");
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
        if (pinpoint == null) {
            telemetry.addLine("NO PINPOINT: this auto cannot run. Check the robot configuration.");
            telemetry.update();
            return;
        }
        pinpoint.update();
        double x = PinpointSetup.forwardInches(pinpoint);   // forward
        double y = PinpointSetup.leftInches(pinpoint);      // left
        double heading = PinpointSetup.headingDegrees(pinpoint); // counterclockwise
        if (gamepad1.dpadDownWasPressed() || gamepad2.dpadDownWasPressed()) { //Recalibration
            pinpoint.resetPosAndIMU();
        }
        if (!SideSelected) {
            if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
                HiveSide = true;
                SideSelected = true;
            }
            if ((gamepad1.bWasPressed() || gamepad2.bWasPressed()) && !SideSelected) {
                HiveSide = false;
                SideSelected = true;
            }
        } else if (SideSelected) {
            if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
                autoMode = MercuryAuto.AutoMode.Leave;
            } else if (gamepad1.bWasPressed() || gamepad2.bWasPressed()) {
                autoMode = MercuryAuto.AutoMode.Park;
            } else if (gamepad1.xWasPressed() || gamepad2.xWasPressed()) {
                autoMode = MercuryAuto.AutoMode.Launch;
            } else if (gamepad1.yWasPressed() || gamepad2.yWasPressed()) {
                autoMode = MercuryAuto.AutoMode.LaunchAndPark;
            } else if (gamepad1.leftBumperWasPressed() || gamepad2.leftBumperWasPressed()) {
                autoMode = MercuryAuto.AutoMode.Intake;
            } else if (gamepad1.rightBumperWasPressed() || gamepad2.rightBumperWasPressed()) {
                SideSelected = false;
                autoMode = null;
            }
        }
        if (SideSelected) {
            telemetry.addLine(HiveSide ? "Hive Side is Selected" : "Wall Side is Selected");
        } else {
            telemetry.addLine("Select Side; A=Hive, B=Wall");
        }

        if (autoMode != null) {
            telemetry.addLine(autoMode + " is Selected");
            telemetry.addLine("RB to reset");
        } else {
            telemetry.addLine("Select Auto Mode; A=Leave, B=Park, X=Launch, Y=Launch And Park, LB=Intake");
        }
        telemetry.addData("Current X", x);
        telemetry.addData("Current Y", y);
        telemetry.addData("Current Heading", heading);
        telemetry.update();
    }
    
    @Override
    public void start() {
        if (autoMode == null) {
            autoMode = MercuryAuto.AutoMode.Park;
        }
        
        switch (autoMode) {
            case Leave:
                queue.add(new Task2(5000, Task2.Type.TRANSLATE, 24, 0)); //Time and distance settings
                break;

            case Park:
                if (HiveSide) {
                    queue.add(new Task2(2000, Task2.Type.TRANSLATE, -12, 12));
                    queue.add(new Task2(4000, Task2.Type.TRANSLATE, -36, 76));
                } else {
                    queue.add(new Task2(2000, Task2.Type.TRANSLATE, 12, 0));
                }
                break;

            case Launch:
                if (HiveSide) {

                } else {

                }
                break;

            case LaunchAndPark:
                if (HiveSide) {

                } else {

                }
                break;
            case Intake:
                if (HiveSide) {

                } else {

                }
        }
        if (pinpoint != null) {
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        }
        timer.reset();
    }
    
    @Override
    public void loop() {
        pinpoint.update();
        double x = PinpointSetup.forwardInches(pinpoint);   // forward
        double y = PinpointSetup.leftInches(pinpoint);      // left
        double heading = PinpointSetup.headingDegrees(pinpoint); // counterclockwise
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
            double xTarget = task2.FieldX;
            double yTarget = task2.FieldY;
            double xError = xTarget - x;
            double yError = yTarget - y;
            double distance = Math.hypot(xError, yError);
            double headingRad = Math.toRadians(heading);
            double sinHeading = Math.sin(headingRad);
            double cosHeading = Math.cos(headingRad);
                double robotStrafeError = -xError * sinHeading + yError * cosHeading;
                double robotForwardError = xError * cosHeading + yError * sinHeading;

            if (distance < DriveTolerance || timer.milliseconds() > task2.end) {
                controls.translationXPower = 0;
                controls.translationYPower = 0;
                controls.rotationPower = 0;
                if (intake != null) {
                    intake.spin(0);
                }
                queue.remove();
                TaskStarted = false;
            }
            else {
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

            if (Math.abs(error) < RotationTolerance || timer.milliseconds() > task2.end) {
                controls.rotationPower = 0;
                queue.remove();
                TaskStarted = false;
            } else {
                controls.rotationPower = rotationPower(error);
            }
            telemetry.addData("Heading Error", error);
            break;
        
        case LAUNCH:
            if (ramp != null && launcher != null) {
                if (!launcherStarted) {
                    launcherTimer.reset();
                    launcherStarted = true;
                }

                if (launcherTimer.milliseconds() < task2.FieldY) {
                    launcher.spin(task2.FieldX);
                } else if (launcherTimer.milliseconds() < task2.end) {
                    launcher.spin(task2.LauncherPower);
                    ramp.spin(1);
                } else {
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
        }
        else
        {
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
        double power = error * rotP;
        return Math.max(-MAX_POWER, Math.min(MAX_POWER, power));
    }
}
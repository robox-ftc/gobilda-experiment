package org.firstinspires.ftc.teamcode;

//import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.devices.BatteryCompensation;
import org.firstinspires.ftc.teamcode.devices.Drivetrain;
import org.firstinspires.ftc.teamcode.devices.DrivetrainControls;
import org.firstinspires.ftc.teamcode.devices.Intake;
import org.firstinspires.ftc.teamcode.devices.Launcher;
import org.firstinspires.ftc.teamcode.devices.Ramp;
import org.firstinspires.ftc.teamcode.devices.Task;

import java.util.*;
/**
 * The time-based autonomous: the same routes as {@link BIOBUZZAutoOdo}, driven
 * by time instead of measured with the Pinpoint - the backup for when the
 * odometry cannot be trusted.
 *
 * <p>Each route is a list of waypoints in inches (right, forward) from where
 * the robot starts, and headings in degrees (counterclockwise, 0 = the way it
 * starts facing). {@link #driveTo} and {@link #turnTo} turn each leg into a
 * power and a time from the robot's measured speeds, keeping track of where
 * the robot should be and which way it should face, so a leg after a turn
 * still goes the right way across the field.
 *
 * <p>Timing only works if a power means a speed. The drive encoders are not
 * connected, so {@link Drivetrain} runs RUN_WITHOUT_ENCODER (otherwise every
 * power is full power), and every drive power is corrected for the battery
 * ({@link BatteryCompensation}). Expect an inch or two of error per leg from
 * wheel slip and speeding up; that is what the odometry auto is for.
 */
@Autonomous(name = "HiveKnightAuto-TimeBased", group = "Robot")
public class BIOBUZZAuto extends OpMode {

    /*
     * The robot's speeds at full power and 12 V. NOT YET MEASURED: these are
     * the test UI's assumed speeds, so the routes come out right there. Run
     * "Cal 3: Drive Speed" on the robot, on open floor, and copy in the
     * numbers on its "For BIOBUZZAuto" line.
     */
    static final double FORWARD_IPS = 56.8; // inches per second driving forward at power 1
    static final double STRAFE_IPS = 56.8;  // inches per second sideways at power 1 (usually less than forward)
    static final double TURN_DPS = 225;     // degrees per second turning at power 1

    /** The power every leg drives and turns at. Lower is more repeatable, higher is quicker. */
    static final double DRIVE_POWER = 0.5;
    static final double TURN_POWER = 0.4;

    /**
     * How long to let the robot come to rest after each leg and turn. Going
     * straight from one leg into the next (worst of all, reversing) carries
     * the last leg's momentum into it: about 3 in lost per reversal here.
     */
    static final int SETTLE_MS = 120;

    /** Launch settings, as tuned on the robot (unaffected by the drive problems). */
    static final int LAUNCH_MS = 6000, SPIN_UP_MS = 3000;

    /** Where the robot should be as the route is built: inches right and forward of the start, degrees CCW. */
    private double planRight, planForward, planHeading;
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
    private BatteryCompensation battery;
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
        battery = new BatteryCompensation(hardwareMap);
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
            autoMode = AutoMode.LaunchAndPark;
        }
        planRight = 0;
        planForward = 0;
        planHeading = 0;

        // The same waypoints as BIOBUZZAutoOdo: inches (right, forward) from the
        // start, headings in degrees counterclockwise.
        switch (autoMode) {
            case Leave:
                driveTo(0, 24);
                break;

            case Park:
                if (HiveSide) {
                    driveTo(-12, 12);
                    driveTo(-36, 76);
                } else {
                    driveTo(0, 6);
                }
                break;

            case Launch:
                if (HiveSide) {
                    launch(0.85);
                } else {
                    wallSideToLaunch();
                    launch(0.8);
                }
                break;

            case LaunchAndPark:
                if (HiveSide) {
                    launch(0.85);
                    driveTo(-12, 12);
                    driveTo(-46, 96);
                } else {
                    wallSideToLaunch();
                    launch(0.8);
                    driveTo(88, 24);
                    driveTo(0, 6);
                }
                break;

            case Intake:
                if (HiveSide) {
                    launch(0.9);
                    driveTo(0, 6);
                    turnTo(90);
                    driveTo(-48, 6);
                    driveTo(0, 6);
                    turnTo(0);
                    launch(0.9);
                    driveTo(-12, 12);
                    driveTo(-46, 96);
                } else {
                    wallSideToLaunch();
                    launch(0.8);
                    driveTo(94, 36);
                    turnTo(0);
                    driveTo(94, 0);
                    driveTo(94, 36);
                    turnTo(-90);
                    launch(0.8);
                    driveTo(88, 24);
                    driveTo(0, 6);
                }
        }
        timer.reset();
    }

    /** From the Wall side start to where it launches from, facing right. */
    private void wallSideToLaunch() {
        driveTo(0, 12);
        driveTo(72, 12);
        driveTo(72, 36);
        turnTo(-90);
        driveTo(100, 36);
    }

    /**
     * Queues a straight leg to (right, forward) inches from the start, at
     * DRIVE_POWER, for as long as the measured speeds say it takes. The leg is
     * worked out in the robot's own frame from the heading it should have, and
     * split between strafing and driving forward so that the two together use
     * DRIVE_POWER (the drivetrain would scale down anything more).
     */
    void driveTo(double right, double forward) {
        double dRight = right - planRight;
        double dForward = forward - planForward;
        planRight = right;
        planForward = forward;
        double h = Math.toRadians(planHeading);
        // The field leg, in the robot's frame: its forward is (-sin h, cos h), its right (cos h, sin h).
        double robotForward = -dRight * Math.sin(h) + dForward * Math.cos(h);
        double robotRight = dRight * Math.cos(h) + dForward * Math.sin(h);
        double seconds = (Math.abs(robotRight) / STRAFE_IPS + Math.abs(robotForward) / FORWARD_IPS) / DRIVE_POWER;
        if (seconds < 0.01) return;
        double xPower = robotRight / (STRAFE_IPS * seconds);
        double yPower = robotForward / (FORWARD_IPS * seconds);
        queue.add(new Task(0, (int) Math.round(seconds * 1000), Task.Type.TRANSLATE, xPower, yPower));
        settle();
    }

    /** Queues a turn on the spot to face {@code heading} degrees (counterclockwise from the start). */
    void turnTo(double heading) {
        double turn = AngleUnit.normalizeDegrees(heading - planHeading);
        planHeading = AngleUnit.normalizeDegrees(heading);
        double seconds = Math.abs(turn) / (TURN_DPS * TURN_POWER);
        if (seconds < 0.01) return;
        // The drivetrain turns clockwise for positive rotation power.
        queue.add(new Task(0, (int) Math.round(seconds * 1000), Task.Type.ROTATE, -Math.signum(turn) * TURN_POWER));
        settle();
    }

    /** A pause with the wheels stopped (a turn at zero power), so the next leg starts from rest. */
    private void settle() {
        queue.add(new Task(0, SETTLE_MS, Task.Type.ROTATE, 0));
    }

    private void launch(double power) {
        queue.add(new Task(0, LAUNCH_MS, Task.Type.LAUNCH, power, SPIN_UP_MS));
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
                    if (timer.milliseconds() >= task.end) { // >=: a leg runs its time, not its time plus a loop
                        controls.translationXPower = 0;
                        controls.translationYPower = 0;
                        if (intake != null) {
                            intake.spin(0);
                        }
                        queue.remove();
                        TaskStarted = false;
                    } else {
                        double k = battery.factor();
                        controls.translationXPower = pClamp(x * k);
                        controls.translationYPower = pClamp(y * k);
                        if (intake != null) {
                            intake.spin(1);
                        }
                    }
                    break;

                case ROTATE:
                    double error = task.target;
                    if (error > 180) error -= 360;
                    if (error < -180) error += 360;
                    if (timer.milliseconds() >= task.end) { // >=: a leg runs its time, not its time plus a loop
                        controls.rotationPower = 0;
                        queue.remove();
                        TaskStarted = false;
                    } else {
                        controls.rotationPower = pClamp(task.target * battery.factor());
                    }
                    break;

                case LAUNCH:
                    if (intake != null && launcher != null) {

                        if (!launcherStarted) {
                            launcherTimer.reset();
                            SpinUpTimer.reset();
                            launcherStarted = true;
                        }

                        if (launcherTimer.milliseconds() < task.target2) {
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
            if (intake != null) intake.spin(0);
            if (launcher != null) launcher.spin(0);
            if (ramp != null) ramp.spin(0);

            if (drivetrain != null) {
                controls.translationXPower = 0;
                controls.translationYPower = 0;
                controls.rotationPower = 0;
                drivetrain.run(controls);
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
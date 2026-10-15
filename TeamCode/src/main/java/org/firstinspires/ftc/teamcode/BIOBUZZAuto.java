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
import org.firstinspires.ftc.teamcode.devices.Task;
import org.firstinspires.ftc.teamcode.devices.Turret;

import java.util.*;
@Autonomous(name = "BIOBUZZAuto", group = "Robot")
public class BIOBUZZAuto extends OpMode {
    private static final double TOLERANCE = 1; // Arbitrary
    private static final double RTOLERANCE = 5; // Arbitrary

    // Tunable P
    private final double P = 1.0 / 20.0;
    // Rotation Factor
    private final double R = 2 * 12.0 / 360.0;
    private boolean launcherStarted;

    private GoBildaPinpointDriver pinpoint;
    private Drivetrain drivetrain;
    private Intake intake;
    private Launcher launcher;
    private Turret turret;
    private DrivetrainControls controls;
    private boolean HiveSide;
    private boolean SideSelected;
    private Queue<Task> queue;
    private ElapsedTime timer;
    ElapsedTime launcherTimer = new ElapsedTime();
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
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(0, 7.5, DistanceUnit.INCH); //Likely needs to be changed
        pinpoint.resetPosAndIMU();
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
        launcherStarted = false;
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
        }
        telemetry.update();
    }
    @Override
    public void start() {
        switch (autoMode) { //All values are placeholders as we do not know the correct ones
            case Leave:
                if (HiveSide) {
                    queue.add(new Task(0, 5000, Task.Type.TRANSLATE, 0, 10));
                } else {
                    // Leave, non-Hive side
                }
                break;

            case Park:
                if (HiveSide) {
                    queue.add(new Task(0, 5000, Task.Type.TRANSLATE, 0, 10));
                    queue.add(new Task(0, 5000, Task.Type.TRANSLATE, 20, 0));
                } else {
                    // Park, non-Hive side
                }
                break;

            case Launch:
                if (HiveSide) {
                    queue.add(new Task(0, 5000, Task.Type.TRANSLATE, 0, 10));
                    queue.add(new Task(0, 5000, Task.Type.LAUNCH, 0, 10));
                } else {
                    // Launch, non-Hive side
                }
                break;

            case LaunchAndPark:
                if (HiveSide) {
                    // LaunchAndPark, Hive side
                } else {
                    // LaunchAndPark, non-Hive side
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

            pinpoint.update();
            double x = pinpoint.getPosX(DistanceUnit.INCH);
            double y = pinpoint.getPosY(DistanceUnit.INCH);
            double heading = pinpoint.getHeading(AngleUnit.DEGREES);

            if (!queue.isEmpty()) {
                Task task = queue.peek();

                switch (task.type) {
                    case TRANSLATE:
                        double errorX = task.target - x;
                        double errorY = task.target2 - y;
                        // square distance, unlikely it'll reach the end of the task
                        if (Math.hypot(errorX, errorY) <= TOLERANCE || timer.milliseconds() > task.end) {
                            queue.remove();
                        } else {
                            controls.translationXPower = pClamp(errorX);
                            controls.translationYPower = pClamp(errorY);
                        }
                        if (intake != null) {
                            intake.spin(1);
                        }
                        break;
                    case ROTATE:
                        double error = task.target - heading;
                        if (error > 180) error -= 360;
                        if (error < -180) error += 360;
                        // Try to avoid the robot turning more than 360
                        if (Math.abs(error) <= RTOLERANCE || timer.milliseconds() > task.end) {
                            queue.remove();
                        } else {
                            controls.rotationPower = pClamp(error * R);
                        }
                        if (intake != null) {
                            intake.spin(0);
                        }
                        break;
                    case LAUNCH:
                        if (intake != null && launcher != null && !launcherStarted) {
                            launcherTimer.reset();
                            launcherStarted = true;
                        }
                        if (launcherStarted && launcherTimer.seconds() < task.target2) {
                            launcher.spin(task.target);
                        }
                        else {
                            assert launcher != null;
                            launcher.stopSpin();
                            queue.remove();
                            launcherStarted = false;
                        }
                        break;

                    default:
                        controls.translationXPower = 0;
                        controls.translationYPower = 0;
                        controls.rotationPower = 0;
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

            telemetry.addData("Current X", x);
            telemetry.addData("Current Y", y);
            telemetry.addData("Current Heading", heading);
            telemetry.addData("Remaining Tasks", queue.size());
            telemetry.update();
        }


        private double pClamp(double error) {
            return Math.min(Math.max(error * P, -1), 1);
        }
    }
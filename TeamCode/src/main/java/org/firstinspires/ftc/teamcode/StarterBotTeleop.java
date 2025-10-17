
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Utils.*;

@TeleOp(name = "StarterBotTeleop", group = "StarterBot")
//@Disabled
public class StarterBotTeleop extends OpMode {
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;

    /*
     * When we control our launcher motor, we are using encoders. These allow the control system
     * to read the current speed of the motor and apply more or less power to keep it at a constant
     * velocity. Here we are setting the target, and minimum velocity that the launcher should run
     * at. The minimum velocity is a threshold for determining when to fire.
     */

    private Launcher launcher = null;
    private LauncherControls launcherControls = null;
    private Drivetrain drivetrain = null;
    private DrivetrainControls drivetrainControls = null;
    private Autonomous auto = null;
    private Intake intake = null;

    private boolean autoMode = true;
    private boolean modeButtonDown = false;

    /*
     * TECH TIP: State Machines
     * We use a "state machine" to control our launcher motor and feeder servos in this program.
     * The first step of a state machine is creating an enum that captures the different "states"
     * that our code can be in.
     * The core advantage of a state machine is that it allows us to continue to loop through all
     * of our code while only running specific code when it's necessary. We can continuously check
     * what "State" our machine is in, run the associated code, and when we are done with that step
     * move on to the next state.
     * This enum is called the "LaunchState". It reflects the current condition of the shooter
     * motor and we move through the enum when the user asks our code to fire a shot.
     * It starts at idle, when the user requests a launch, we enter SPIN_UP where we get the
     * motor up to speed, once it meets a minimum speed then it starts and then ends the launch process.
     * We can use higher level code to cycle through these states. But this allows us to write
     * functions and autonomous routines in a way that avoids loops within loops, and "waits".
     */

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
         drivetrain = new Drivetrain(hardwareMap);
         drivetrainControls = new DrivetrainControls();
         launcher = new Launcher(hardwareMap, telemetry);
         launcherControls = new LauncherControls();
         intake = new Intake(hardwareMap);
         auto = new Autonomous(drivetrainControls, launcherControls, telemetry);
        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        /*
         * Here we call a function called arcadeDrive. The arcadeDrive function takes the input from
         * the joysticks, and applies power to the left and right drive motor to move the robot
         * as requested by the driver. "arcade" refers to the control style we're using here.
         * Much like a classic arcade game, when you move the left joystick forward both motors
         * work to drive the robot forward, and when you move the right joystick left and right
         * both motors work to rotate the robot. Combinations of these inputs can be used to create
         * more complex maneuvers.
         */
        // Sensing
        boolean newModeButtonDown = gamepad1.y || gamepad2.y;
        if (modeButtonDown && !newModeButtonDown) {
            autoMode = !autoMode; // we toggle auto mode when the button is switch from pressed to up.
        }
        this.modeButtonDown = newModeButtonDown;

        double intakeSpeed = autoMode ? 1.0 : readIntakeSpeed(gamepad1, gamepad2);

        // planning
        if (autoMode && auto.status) {
            auto.run();
            if (!auto.status) {
                autoMode = false;
            }
        } else {
            drivetrainControls = readDrivetrainControls(gamepad1, gamepad2);
            launcherControls = readLauncherControls(gamepad1, gamepad2);
        }
        intake.setPower(intakeSpeed);
        telemetry.addData("translationY", drivetrainControls.translationY);
        telemetry.addData("rotation", drivetrainControls.rotation);
        drivetrain.setPowers(computeDriveTrainPower(drivetrainControls));
        // execution
        drivetrain.run();
        launcher.manualLaunch(launcherControls);
        intake.spin();

        /*
         * Show the state and motor powers
         */
        telemetry.addData("mode", autoMode);
        telemetry.addData("status", auto.status);
        telemetry.addData("trigger", launcherControls.trigger);
        telemetry.addData("intake", intakeSpeed);
        telemetry.addData("feederPosition", launcher.getFeederAngle());
    }

    private double readIntakeSpeed(Gamepad gamepad1, Gamepad gamepad2) {
        return gamepad1.rightBumperWasPressed() || gamepad2.rightBumperWasPressed() ? 1.0 : Math.max(gamepad1.right_trigger, gamepad2.right_trigger);
    }

    private DrivetrainControls readDrivetrainControls(Gamepad gamepad1, Gamepad gamepad2) {
        double x = gamepad1.left_stick_x + gamepad2.left_stick_x;
        // Notes: stick's y positive direction is pointing down (toward player).
        double y = -(gamepad1.left_stick_y + gamepad2.left_stick_y);
        double a = gamepad1.right_stick_x + gamepad2.right_stick_x;
        return new DrivetrainControls(x, y, a);
    }

    private LauncherControls readLauncherControls(Gamepad gamepad1, Gamepad gamepad2) {
        double wheelPress = Math.max(gamepad1.left_trigger, gamepad2.left_trigger);
        return new LauncherControls(wheelPress);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }


    private double[] computeDriveTrainPower(DrivetrainControls controls) {

        double frontLeftPower  = controls.translationY + controls.translationX + controls.rotation;
        double frontRightPower = controls.translationY - controls.translationX - controls.rotation;
        double rearLeftPower   = controls.translationY - controls.translationX + controls.rotation;
        double rearRightPower  = controls.translationY + controls.translationX - controls.rotation;

        double maxPower = Math.max(1.0, Math.max(
                Math.abs(frontLeftPower),
                Math.max(Math.abs(frontRightPower),
                        Math.max(Math.abs(rearLeftPower),
                                Math.abs(rearRightPower)))
        ));

        frontLeftPower  /= maxPower;
        frontRightPower /= maxPower;
        rearRightPower  /= maxPower;
        rearLeftPower   /= maxPower;

        return new double[]{frontLeftPower, frontRightPower, rearRightPower, rearLeftPower};
    }
}
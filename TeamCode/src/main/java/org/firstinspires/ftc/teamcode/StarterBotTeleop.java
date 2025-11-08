
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utils.*;

@TeleOp(name = "StarterBotTeleop-2025Decode", group = "StarterBot")
//@Disabled
public class StarterBotTeleop extends OpMode {
    /*
     * When we control our launcher motor, we are using encoders. These allow the control system
     * to read the current speed of the motor and apply more or less power to keep it at a constant
     * velocity. Here we are setting the target, and minimum velocity that the launcher should run
     * at. The minimum velocity is a threshold for determining when to fire.
     */

    private Launcher launcher = null;
    private Drivetrain drivetrain = null;
    private Intake intake = null;
    public GamePadReadings oldGamePadReadings = new GamePadReadings();
    private boolean autoMode = false;

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
         drivetrain = new Drivetrain(hardwareMap, telemetry, false);
         launcher = new Launcher(hardwareMap, telemetry);
         intake = new Intake(hardwareMap, telemetry);
         telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        //launcher.homeTurretInitLoop();
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
        // This is the global readings.
        GamePadReadings newGamepadReadings = new GamePadReadings(){{
            bButton = gamepad1.b || gamepad2.b;
            aButton = gamepad1.a || gamepad2.a;
            xButton = gamepad1.x || gamepad2.x;
            yButton = gamepad1.y || gamepad2.y;
            leftBumper = gamepad1.left_bumper || gamepad2.left_bumper;
            rightBumper = gamepad1.right_bumper || gamepad2.right_bumper;

            leftStickX = gamepad1.left_stick_x;
            leftStickY = gamepad1.left_stick_y;
            rightStickX = gamepad1.right_stick_x;
            rightStickY = gamepad1.right_stick_y;

            leftTrigger = Math.max(gamepad1.left_trigger, gamepad2.left_trigger);
            rightTrigger = Math.max(gamepad1.right_trigger, gamepad2.right_trigger);

            dPadUp = gamepad1.dpad_up || gamepad2.dpad_up;
            dPadDown = gamepad1.dpad_down || gamepad2.dpad_down;
        }};

        autoMode = Utils.toggle(autoMode, Utils.buttonUp(oldGamePadReadings.yButton, newGamepadReadings.yButton));
        telemetry.addData("mode", autoMode);

        intake.readControls(oldGamePadReadings, newGamepadReadings);
        launcher.readControls(oldGamePadReadings, newGamepadReadings);
        drivetrain.readControls(oldGamePadReadings, newGamepadReadings);

        ///  Actions
        intake.run(autoMode);
        launcher.run(autoMode);
        drivetrain.run(autoMode);

        this.oldGamePadReadings = newGamepadReadings;
    }
}
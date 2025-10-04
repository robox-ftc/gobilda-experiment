package org.firstinspires.ftc.teamcode.Old;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import java.util.Locale;

@TeleOp(name = "Mecanum Double Controller", group = "Robot")
// @Disabled
public class Mecanum extends LinearOpMode {
    /* Declare OpMode members. */
    public DcMotor LFDrive = null; // left-front
    public DcMotor RFDrive = null; // right-front
    public DcMotor LBDrive = null; // left-back
    public DcMotor RBDrive = null; // right-front
    public DcMotor armMotor = null; // the arm motor
    public CRServo intake = null; // the active intake servo
    public Servo wrist = null; // the wrist servo
    public IMU imu = null; //

    final double MOTOR_SPEED = 1;
    final double STRAFE_SPEED = 1; // adjust
    GoBildaPinpointDriver odo;
    double oldTime = 0;
    /*
     * This constant is the number of encoder ticks for each degree of rotation of
     * the arm.
     * To find this, we first need to consider the total gear reduction powering our
     * arm.
     * First, we have an external 20t:100t (5:1) reduction created by two spur
     * gears.
     * But we also have an internal gear reduction in our motor.
     * The motor we use for this arm is a 117RPM Yellow Jacket. Which has an
     * internal gear
     * reduction of ~50.9:1. (more precisely it is 250047/4913:1)
     * We can multiply these two ratios together to get our final reduction of
     * ~254.47:1.
     * The motor's encoder counts 28 times per rotation. So in total you should see
     * about 7125.16
     * counts per rotation of the arm. We divide that by 360 to get the counts per
     * degree.
     */
    final double ARM_TICKS_PER_DEGREE = 28 // number of encoder ticks per rotation of the bare motor
            * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
            * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T
            // hub-mount gear
            * 1 / 360.0; // we want ticks per degree, not per rotation
    final double ARM_COLLAPSED_INTO_ROBOT = 0;
    final double ARM_COLLECT = 250 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER = 230 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_LOW = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_ATTACH_HANGING_HOOK = 120 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT = 180 * ARM_TICKS_PER_DEGREE;

    /*
     * Variables to store the speed the intake servo should be set at to intake, and
     * deposit game elements.
     */
    final double INTAKE_COLLECT = -1.0;
    final double INTAKE_OFF = 0.0;
    final double INTAKE_DEPOSIT = 0.5;

    /*
     * Variables to store the positions that the wrist should be set to when folding
     * in, or folding out.
     */
    final double WRIST_FOLDED_IN = 0.4667; // adjusted due to intake problem
    final double WRIST_FOLDED_OUT = 0.8333; // adjusted due to intake problem

    /* A number in degrees that the triggers can adjust the arm position by */
    final double FUDGE_FACTOR = ARM_TICKS_PER_DEGREE * 180; // near-continuous movement
    /* Variables that are used to set the arm to a specific position */
    double armPosition = (int) ARM_COLLAPSED_INTO_ROBOT;
    double armPositionFudgeFactor;

    @Override
    public void runOpMode() {
        /*
         * These variables are private to the OpMode, and are used to control the
         * drivetrain.
         */
        double leftFront;
        double rightFront;
        double leftBack;
        double rightBack;
        // double forward;
        // double strafe;
        // double rotate;
        double max;

        /* Define and Initialize Motors */
        LFDrive = hardwareMap.get(DcMotor.class, "fl"); // the left drivetrain motor
        RFDrive = hardwareMap.get(DcMotor.class, "fr"); // the right drivetrain motor
        LBDrive = hardwareMap.get(DcMotor.class, "bl"); // the left drivetrain motor
        RBDrive = hardwareMap.get(DcMotor.class, "br"); // the right drivetrain motor
        armMotor = hardwareMap.get(DcMotor.class, "arm"); // the arm motor
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        /*
         * Most skid-steer/differential drive robots require reversing one motor to
         * drive forward.
         * for this robot, we reverse the right motor.
         */
        LFDrive.setDirection(DcMotor.Direction.REVERSE);
        RFDrive.setDirection(DcMotor.Direction.FORWARD);
        LBDrive.setDirection(DcMotor.Direction.REVERSE);
        RBDrive.setDirection(DcMotor.Direction.FORWARD);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the
         * motor to slow down
         * much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot
         * stops much quicker.
         */
        LFDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RFDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*
         * This sets the maximum current that the control hub will apply to the arm
         * before throwing a flag
         */
        ((DcMotorEx) armMotor).setCurrentAlert(5, CurrentUnit.AMPS);

        /*
         * Before starting the armMotor. We'll make sure the TargetPosition is set to 0.
         * Then we'll set the RunMode to RUN_TO_POSITION. And we'll ask it to stop and
         * reset encoder.
         * If you do not have the encoder plugged into this motor, it will not run in
         * this code.
         */
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /* Define and initialize servos. */
        intake = hardwareMap.get(CRServo.class, "intake");
        wrist = hardwareMap.get(Servo.class, "wrist");
        /* Make sure that the intake is off, and the wrist is folded in. */
        intake.setPower(INTAKE_OFF);
        wrist.setPosition(WRIST_FOLDED_IN);

        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready.");
        telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
        telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.MM));
        telemetry.addData("Heading Scalar", odo.getYawScalar());
        telemetry.update();

        // Describe how the Hub is mounted
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);

        waitForStart();

        /* Run until the driver presses stop */
        while (opModeIsActive()) {
            odo.update();

            /*
             * Optionally, you can update only the heading of the device. This takes less
             * time to read, but will not
             * pull any other data. Only the heading (which you can pull with getHeading()
             * or in getPosition().
             */
            // odo.update(GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING);

            if (gamepad1.a) {
                odo.resetPosAndIMU(); // resets the position to 0 and recalibrates the IMU
            }

            if (gamepad1.b) {
                odo.recalibrateIMU(); // recalibrates the IMU without resetting position
            }

            /*
             * This code prints the loop frequency of the REV Control Hub. This frequency is
             * effected
             * by I²C reads/writes. So it's good to keep an eye on. This code calculates the
             * amount
             * of time each cycle takes and finds the frequency (number of updates per
             * second) from
             * that cycle time.
             */
            double newTime = getRuntime();
            double loopTime = newTime - oldTime;
            double frequency = 1 / loopTime;
            oldTime = newTime;

            /*
             * gets the current Position (x & y in mm, and heading in degrees) of the robot,
             * and prints it.
             */
            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM),
                    pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));

            /*
             * gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and
             * prints it.
             */
            String velocity = String.format(Locale.US, "{XVel: %.3f, YVel: %.3f, HVel: %.3f}",
                    odo.getVelX(DistanceUnit.MM), odo.getVelY(DistanceUnit.MM),
                    odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));

            /*
             * Gets the Pinpoint device status. Pinpoint can reflect a few states. But we'll
             * primarily see
             * READY: the device is working as normal
             * CALIBRATING: the device is calibrating and outputs are put on hold
             * NOT_READY: the device is resetting from scratch. This should only happen
             * after a power-cycle
             * FAULT_NO_PODS_DETECTED - the device does not detect any pods plugged in
             * FAULT_X_POD_NOT_DETECTED - The device does not detect an X pod plugged in
             * FAULT_Y_POD_NOT_DETECTED - The device does not detect a Y pod plugged in
             * FAULT_BAD_READ - The firmware detected a bad I²C read, if a bad read is
             * detected, the device status is updated and the previous position is reported
             */

            leftFront = (gamepad1.left_stick_y - gamepad1.left_stick_x) * STRAFE_SPEED;
            rightFront = (gamepad1.right_stick_y + gamepad1.right_stick_x) * STRAFE_SPEED;
            leftBack = (gamepad1.left_stick_y + gamepad1.left_stick_x) * STRAFE_SPEED;
            rightBack = (gamepad1.right_stick_y - gamepad1.right_stick_x) * STRAFE_SPEED;

            /* Normalize the values so neither exceed +/- 1.0 */
            max = Math.max(Math.max(Math.abs(leftFront), Math.abs(rightFront)),
                    Math.max(Math.abs(leftBack), Math.abs(rightBack)));
            if (max > 1.0) {
                leftFront /= max;
                rightFront /= max;
                leftBack /= max;
                rightBack /= max;
            }

            /* Set the motor power to the variables we've mixed and normalized */
            LFDrive.setPower(leftFront * MOTOR_SPEED);
            RFDrive.setPower(rightFront * MOTOR_SPEED);
            LBDrive.setPower(leftBack * MOTOR_SPEED);
            RBDrive.setPower(rightBack * MOTOR_SPEED);

            /*
             * Here we handle the three buttons that have direct control of the intake
             * speed.
             * These control the continuous rotation servo that pulls elements into the
             * robot,
             * If the user presses A, it sets the intake power to the final variable that
             * holds the speed we want to collect at.
             * If the user presses X, it sets the servo to Off.
             * And if the user presses B it reveres the servo to spit out the element.
             */

            /*
             * TECH TIP: If Else statements:
             * We're using an else if statement on "gamepad1.x" and "gamepad1.b" just in
             * case
             * multiple buttons are pressed at the same time. If the driver presses both "a"
             * and "x"
             * at the same time. "a" will win over and the intake will turn on. If we just
             * had
             * three if statements, then it will set the intake servo's power to multiple
             * speeds in
             * one cycle. Which can cause strange behavior.
             */

            if (gamepad2.a || gamepad1.a) {
                intake.setPower(INTAKE_COLLECT);
            } else if (gamepad2.x || gamepad1.x) {
                intake.setPower(INTAKE_OFF);
            } else if (gamepad2.b || gamepad1.b) {
                intake.setPower(INTAKE_DEPOSIT);
            }

            /*
             * Here we implement a set of if else statements to set our arm to different
             * scoring positions.
             * We check to see if a specific button is pressed, and then move the arm (and
             * sometimes
             * intake and wrist) to match. For example, if we click the right bumper we want
             * the robot
             * to start collecting. So it moves the armPosition to the ARM_COLLECT position,
             * it folds out the wrist to make sure it is in the correct orientation to
             * intake, and it
             * turns the intake on to the COLLECT mode.
             */

            if (gamepad2.right_bumper || gamepad1.right_bumper) {
                /* This is the intaking/collecting arm position */
                armPosition = ARM_COLLECT;
                wrist.setPosition(WRIST_FOLDED_OUT);
                intake.setPower(INTAKE_COLLECT);
            }

            else if (gamepad2.left_bumper || gamepad1.left_bumper) {
                /*
                 * This is about 20° up from the collecting position to clear the barrier
                 * Note here that we don't set the wrist position or the intake power when we
                 * select this "mode", this means that the intake and wrist will continue what
                 * they were doing before we clicked left bumper.
                 */
                armPosition = ARM_CLEAR_BARRIER;
            }

            else if (gamepad2.y || gamepad1.y) {
                /* This is the correct height to score the sample in the LOW BASKET */
                wrist.setPosition(WRIST_FOLDED_OUT);
                armPosition = ARM_SCORE_SAMPLE_IN_LOW;
            }

            else if (gamepad2.dpad_left || gamepad1.dpad_left) {
                /*
                 * This turns off the intake, folds in the wrist, and moves the arm
                 * back to folded inside the robot. This is also the starting configuration
                 */
                armPosition = ARM_COLLAPSED_INTO_ROBOT;
                intake.setPower(INTAKE_OFF);
                wrist.setPosition(WRIST_FOLDED_IN);
            }

            else if (gamepad2.dpad_right || gamepad1.dpad_right) {
                /* This is the correct height to score SPECIMEN on the HIGH CHAMBER */
                armPosition = ARM_SCORE_SPECIMEN;
                wrist.setPosition(WRIST_FOLDED_IN);
            }

            else if (gamepad2.dpad_up || gamepad1.dpad_up) {
                /* This sets the arm to vertical to hook onto the LOW RUNG for hanging */
                armPosition = ARM_ATTACH_HANGING_HOOK;
                intake.setPower(INTAKE_OFF);
                wrist.setPosition(WRIST_FOLDED_IN);
            }

            else if (gamepad2.dpad_down || gamepad1.dpad_down) {
                /* this moves the arm down to lift the robot up once it has been hooked */
                armPosition = ARM_WINCH_ROBOT;
                intake.setPower(INTAKE_OFF);
                wrist.setPosition(WRIST_FOLDED_IN);
            }

            /*
             * Here we create a "fudge factor" for the arm position.
             * This allows you to adjust (or "fudge") the arm position slightly with the
             * gamepad triggers.
             * We want the left trigger to move the arm up, and right trigger to move the
             * arm down.
             * So we add the right trigger's variable to the inverse of the left trigger. If
             * you pull
             * both triggers an equal amount, they cancel and leave the arm at zero. But if
             * one is larger
             * than the other, it "wins out". This variable is then multiplied by our
             * FUDGE_FACTOR.
             * The FUDGE_FACTOR is the number of degrees that we can adjust the arm by with
             * this function.
             */

            armPositionFudgeFactor = FUDGE_FACTOR * (gamepad2.right_trigger + (-gamepad2.left_trigger));

            /*
             * Here we set the target position of our arm to match the variable that was
             * selected
             * by the driver.
             * We also set the target velocity (speed) the motor runs at, and use setMode to
             * run it.
             */

            armMotor.setTargetPosition((int) (armPosition + armPositionFudgeFactor));

            ((DcMotorEx) armMotor).setVelocity(1000);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (((DcMotorEx) armMotor).isOverCurrent()) {
                telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
            }

            /*
             * send telemetry to the driver of the arm's current position and target
             * position
             */
            telemetry.addData("Status", odo.getDeviceStatus());
            telemetry.addData("armTarget: ", armMotor.getTargetPosition());
            telemetry.addData("arm Encoder: ", armMotor.getCurrentPosition());
            telemetry.addData("Velocity", velocity);
            telemetry.addData("Pinpoint Frequency", odo.getFrequency()); // prints/gets the current refresh rate of the
                                                                         // Pinpoint
            telemetry.addData("Position", data);
            telemetry.addData("REV Hub Frequency: ", frequency); // prints the control system refresh rate
            telemetry.update();
        }
    }
}
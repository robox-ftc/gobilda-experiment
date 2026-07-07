package org.firstinspires.ftc.teamcode.devices;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import org.firstinspires.ftc.teamcode.data.AimingParameters;
import org.firstinspires.ftc.teamcode.data.Target;
import org.firstinspires.ftc.teamcode.utils.Utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Launcher {
    public static double LAUNCHER_TICKS_PER_REV = 154.1; // for 1150 RPM motor
    public static double LAUNCHER_MAX_VELOCITY_RPM = 1150;
    public static double LAUNCHER_MAX_VELOCITY_DPS = LAUNCHER_MAX_VELOCITY_RPM / 60.0;
    public static double LAUNCHER_MIN_VELOCITY_RPM = 1075;
    public static double LAUNCHER_MIN_VELOCITY_DPS = LAUNCHER_MIN_VELOCITY_RPM / 60.0;
    public static double ERROR_RATE = 0.1;
    private static final double TURRET_TICKS_PER_REV = 5281.0; // for GoBILDA  30RPM
    // This should be the ticks per each degree of turret change.
    // Motor ticks per revolution / 360 * worm-gear-ratio (1:28)
    private static final double TURRET_TICKS_PER_DEGREE = TURRET_TICKS_PER_REV/360*28;
    public static double FEEDER_ANGLE_SPAN = 300.0; // 300.0 // for goblida 2000-2500-0002
    final double STOP_POWER = 0.0; //We send this power to the servos when we want them to stop.

    private double feederFireAngle;
    private double feederReloadAngle;
    public static double FEED_TIME_SECONDS = 0.20; // The feeder servos run this long when a shot is requested.

    public enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
        LAUNCHED,
        ABORTING
    }

    public LaunchState launchState = LaunchState.IDLE;
    private DcMotorEx launcherLeft = null;
    private DcMotorEx launcherRight = null;
    private DigitalChannel turretHomeSwitch = null;
    private Servo feeder = null;
    ElapsedTime feederTimer = new ElapsedTime();
    Telemetry telemetry;
    public Launcher(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        initShooter(hardwareMap);
        initFeeder(hardwareMap);
    }

    public boolean isTurretHomed() {
        // flip this flag given connections.
        return !turretHomeSwitch.getState();
    }

    public boolean isLauncherSpeedReady(double targetRpm, double toleranceRatio, double diffToleranceRatio) {
        double leftSpeed = this.getLeftWheelRPM();
        double rightSpeed = this.getRightWheelRPM();
        return Math.abs(leftSpeed - targetRpm) / targetRpm < toleranceRatio &&
                Math.abs(rightSpeed - targetRpm) / targetRpm < toleranceRatio &&
                Math.abs(leftSpeed - rightSpeed) * 2 / (leftSpeed + rightSpeed) < diffToleranceRatio;
    }

    public void initShooter(HardwareMap hardwareMap) {
        launcherLeft = hardwareMap.get(DcMotorEx.class, "leftLauncher");
        launcherRight = hardwareMap.get(DcMotorEx.class, "rightLauncher");
        DcMotorEx[] launchers = new DcMotorEx[2];
        launchers[0] = launcherLeft;
        launchers[1] = launcherRight;
        launcherLeft.setDirection(DcMotor.Direction.REVERSE);
        launcherRight.setDirection(DcMotor.Direction.FORWARD);
        // for now, to be changed
        Utils.applyAction(launchers, (motor) -> motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER));
        Utils.applyAction(launchers, (motor) -> motor.setZeroPowerBehavior(FLOAT));
        Utils.applyAction(launchers, (motor) -> motor.setPower(0.0));
    }

    public void run(LauncherControls controls)
    {
        manualLaunch(controls);
    }

    public void initFeeder(HardwareMap hardwareMap) {
        /*
         * Set Feeders to an initial value to initialize the servo controller
         */
        feeder = hardwareMap.get(Servo.class, "feeder");
        // The following setting depends on your hardware mountings.
        this.feederReloadAngle = 0.58;


        this.feederFireAngle = this.feederReloadAngle - 50.0 / FEEDER_ANGLE_SPAN;
        feeder.resetDeviceConfigurationForOpMode();
        feeder.setPosition(feederReloadAngle);
        feederTimer = new ElapsedTime();
    }

    public boolean isFeederLaunched() {
        // adjust the +/- according to servo direction
        return this.feeder.getPosition() > this.feederFireAngle - 10.0 / FEEDER_ANGLE_SPAN;
    }

    public boolean isFeederReset() {
        // adjust the +/- according to servo direction
        return this.feeder.getPosition() < this.feederReloadAngle + 10.0 / FEEDER_ANGLE_SPAN;
    }

    public void spin(double power) {
        launcherLeft.setPower(power);
        launcherRight.setPower(power);
    }

    public void spinToVelocity(double targetSpeedDPS) // degree per second
    {
        launcherLeft.setVelocity(targetSpeedDPS);
        launcherRight.setVelocity(targetSpeedDPS);
    }

    public void stopSpin() {
        spin(STOP_POWER);
    }

    public void fire(double length) {
        feeder.setPosition(this.feederFireAngle * length);
    }

    public void resetFeeder() {
        feeder.setPosition(this.feederReloadAngle);
    }



    public double[] getWheelPRMs(){
        return new double[]{getLeftWheelRPM(), getRightWheelRPM()};
    }

    public static double RRP = 145.1; // for 5.1 : 1 1150 RPM
    public double getLeftWheelRPM(){
        // get velocity returns TPS, ticks per second
        return launcherLeft.getVelocity() / RRP * 60 ;
    }

    public double getRightWheelRPM(){
        return launcherRight.getVelocity() / RRP * 60;
    }

    public void manualLaunch(LauncherControls controls) {
        // debug the relation between ticks and angle, remove later
        telemetry.addData("RPM-left", this.getLeftWheelRPM());
        telemetry.addData("RPM-right", this.getRightWheelRPM());

        launcherLeft.setPower(controls.leftWheelPower);
        launcherRight.setPower(controls.rightWheelPower);

        if (controls.triggerDown)
            fire(calculateAngle());
        else
            resetFeeder();
    }

    public void abort() {
        stopSpin();
        resetFeeder();
        launchState = LaunchState.IDLE;
    }

    // How does a servo motor work?
    // power 0 ~ 1 is mapped to angle 0 to max span (300 degrees for most of servos)
    // We set it at 0.5 for the middle, this is also our initial position.
    // 0.8 means 1/300 * 0.8
    private double calculateAngle() {
        // find a relationship between turret angle and feeder angle.
        return 0.8;
    }
}

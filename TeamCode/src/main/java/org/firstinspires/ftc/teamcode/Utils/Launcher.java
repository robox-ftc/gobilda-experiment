package org.firstinspires.ftc.teamcode.Utils;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import static org.firstinspires.ftc.teamcode.Utils.Utils.applyAction;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Launcher implements IDevice {
    public static double LAUNCHER_MAX_VELOCITY_RPM = 1620;
    public static double LAUNCHER_MAX_VELOCITY_DPS = LAUNCHER_MAX_VELOCITY_RPM / 60.0;
    public static double LAUNCHER_MIN_VELOCITY_RPM = 1075;
    public static double LAUNCHER_MIN_VELOCITY_DPS = LAUNCHER_MIN_VELOCITY_RPM / 60.0;
    public static double ERROR_RATE = 0.1;
    private static final double TURRET_TICKS_PER_REV = 5272.0; // for GoBILDA  30RPM
    private static final double TURRET_TICKS_PER_DEGREE = 5272.0/360;
    public static double FEEDER_ANGLE_SPAN = 300.0; // for goblida 2000-2500-0002
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;

    private double feederFireAngle;
    private double feederReloadAngle;
    public static double FEED_TIME_SECONDS = 0.20; //The feeder servos run this long when a shot is requested.

    public enum Artifact {
        Green,
        Purple
    }

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
    private DcMotorEx turret = null;
    private DigitalChannel turretHomeSwitch = null;
    private Servo feeder = null;

    public double targetSpeed;
    ElapsedTime feederTimer = new ElapsedTime();
    Telemetry telemetry;

    public LauncherControls controls = new LauncherControls();

    public Launcher(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        initShooter(hardwareMap);
        initFeeder(hardwareMap);
        initTurret(hardwareMap);
    }

    public boolean isTurretHomed() {
        boolean switchStatus =  !turretHomeSwitch.getState();
        this.telemetry.addData("Turret limit switch state", switchStatus);
        return switchStatus;
    }

    // To be called in init_loop
    public void homeTurretInitLoop() {
        boolean isHomed = isTurretHomed();
        if (!isHomed) {
            telemetry.addData("Turret Homed", isHomed);
            turret.setPower(-1.0);
        }
        else{
            Utils.stopAndResetEncoder(turret);
            telemetry.addData("Turret position:", turret.getCurrentPosition()/TURRET_TICKS_PER_DEGREE);
        }
    }

    public void initTurret(HardwareMap hardwareMap) {
        turretHomeSwitch = hardwareMap.get(DigitalChannel.class, "turretHomeSwitch");
        turretHomeSwitch.setMode(DigitalChannel.Mode.INPUT);
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setZeroPowerBehavior(FLOAT); // Do not need to break; The gear can be self-locked.
    }

    public boolean isLauncherSpeedReady(double targetSpeed, double toleranceRatio, double diffToleranceRatio) {
        double leftSpeed = launcherLeft.getVelocity(AngleUnit.DEGREES);
        double rightSpeed = launcherRight.getVelocity(AngleUnit.DEGREES);
        return Math.abs(leftSpeed - targetSpeed) / targetSpeed < toleranceRatio &&
                Math.abs(rightSpeed - targetSpeed) / targetSpeed < toleranceRatio &&
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
        applyAction(launchers, (motor) -> motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER));
        applyAction(launchers, (motor) -> motor.setZeroPowerBehavior(FLOAT));
        applyAction(launchers, (motor) -> motor.setPower(0.0));
    }

    public void readControls(GamePadReadings oldReadings, GamePadReadings newReadings)
    {
        this.controls.wheelPower = newReadings.leftBumper ? 1.0 : newReadings.leftTrigger;
        this.controls.triggerDown = oldReadings.aButton;
        this.controls.fireRequested = Utils.buttonUp(oldReadings.aButton, newReadings.aButton);
        this.controls.abortRequested = Utils.buttonUp(oldReadings.bButton, newReadings.bButton);
        // constrain angle from 0 to 1, we want it only goes to one position above the "zero" point.
        this.controls.turretPower = (newReadings.dPadUp ? 1 : 0) - (newReadings.dPadDown ? 1 : 0);

        telemetry.addData("trigger", this.controls.fireRequested);
        telemetry.addData("triggerDown", this.controls.triggerDown);
    }

    @Override
    public void run(boolean autoMode) {
        if (autoMode){
            autoLaunch(this.controls);
        }
        else{
            manualLaunch(this.controls);
        }
    }

    public void initFeeder(HardwareMap hardwareMap) {
        /*
         * set Feeders to an initial value to initialize the servo controller
         */
        feeder = hardwareMap.get(Servo.class, "feeder");

        // The following setting depends on your hardware mountings.
        this.feederReloadAngle = 0.5;
        this.feederFireAngle = this.feederReloadAngle - 75.0 / FEEDER_ANGLE_SPAN;
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
        launcherLeft.setPower(STOP_SPEED);
        launcherRight.setPower(STOP_SPEED);
    }

    public void fire(double length) {
        feeder.setPosition(this.feederFireAngle * length);
    }

    public void resetFeeder() {
        feeder.setPosition(this.feederReloadAngle);
    }

    public double getFeederAngle() {
        return feeder.getPosition();
    }

    public void autoLaunch(LauncherControls controls) {
        if (controls.abortRequested)
            abort();
        else
            launch(controls.fireRequested);
    }

    public void manualLaunch(LauncherControls controls) {
        telemetry.addData("t power", controls.turretPower);
        if (isTurretHomed() && controls.turretPower < 0){
            turret.setPower(0);
        }
        else{
            turret.setPower((controls.turretPower));
        }

        spin(controls.wheelPower);
        if (controls.triggerDown)
            fire(calculateAngle());
        else
            resetFeeder();
    }

    public void Aim() {
    }

    public boolean isAimed() {
        return true;
    }

    public boolean isFeederLoaded() {
        return true;
    }

    public void ReloadAmmo()
    {
    }

    public void abort() {
        stopSpin();
        resetFeeder();
        launchState = LaunchState.IDLE;
    }

    public double calculateSpeed()
    {
        // 1500 rpm to dps
        double dps = 1500 * 60;
        if (controls.turretPower == 1) {
            dps *= 0.8;
        }
        telemetry.addData("target roller dps", dps);
        return dps;
    }

    public void launch(boolean fireRequested) {
        switch (launchState) {
            case IDLE:
                if (fireRequested) {
                    Aim(); // when implement, sending command to other devices to aim and reload but dont wait.
                    ReloadAmmo();
                    launchState = LaunchState.SPIN_UP;
                    this.targetSpeed = calculateSpeed();
                }
                break;
            case SPIN_UP: // For idempotent actions, we can let the machine re enter the same state and check
                spinToVelocity(this.targetSpeed);
                if (isLauncherSpeedReady(this.targetSpeed, 0.9, ERROR_RATE)
                    && isAimed() && isFeederLoaded())
                    launchState = LaunchState.LAUNCH;
                break;
            case LAUNCH:
                fire(calculateAngle());
                feederTimer.reset();
                if (isFeederLaunched()) {
                    resetFeeder();
                    launchState = LaunchState.LAUNCHING;
                }
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS)
                    launchState = LaunchState.LAUNCHED;
                break;
            case LAUNCHED:
                if (isFeederReset())
                    launchState = LaunchState.IDLE;
                stopSpin();
                break;
            default: {
                break;
            }
        }
    }

    private double calculateAngle() {
        if (controls.turretPower == 1) {
            return 1;
        } else {
            return 0.8;
        }
    }
}

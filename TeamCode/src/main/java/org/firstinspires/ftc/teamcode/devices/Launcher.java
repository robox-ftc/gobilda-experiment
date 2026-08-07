package org.firstinspires.ftc.teamcode.devices;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import org.firstinspires.ftc.teamcode.utils.Utils;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Launcher {
    public static double LAUNCHER_TICKS_PER_REV = 103.8; // for 1620 RPM motor
    public static double LAUNCHER_MAX_VELOCITY_RPM = 1620;
    public static double LAUNCHER_MAX_VELOCITY_DPS = LAUNCHER_MAX_VELOCITY_RPM / 60.0;
    public static double ERROR_RATE = 0.1;
    final double STOP_POWER = 0.0; // We send this power to the servos when we want them to stop.

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
    private CRServo feeder = null;
    ElapsedTime timer = new ElapsedTime();
    Telemetry telemetry;
    public Launcher(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        initShooter(hardwareMap);
        initFeeder(hardwareMap);
    }

    public void initShooter(HardwareMap hardwareMap) {
        launcherLeft = hardwareMap.get(DcMotorEx.class, "leftLauncher");
        launcherRight = hardwareMap.get(DcMotorEx.class, "rightLauncher");
        DcMotorEx[] launchers = new DcMotorEx[2];
        launchers[0] = launcherLeft;
        launchers[1] = launcherRight;
        launcherLeft.setDirection(DcMotor.Direction.FORWARD);
        launcherRight.setDirection(DcMotor.Direction.REVERSE);
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
        feeder = hardwareMap.get(CRServo.class, "feeder");
        // The following setting depends on your hardware mountings.
        feeder.resetDeviceConfigurationForOpMode();
        feeder.setPower(0);
        timer = new ElapsedTime();
    }

    public void spin(double power) {
        launcherLeft.setPower(power);
        launcherRight.setPower(power);
    }

    public void spinToVelocity(double targetSpeedTPS) { // ticks per second
        launcherLeft.setVelocity(targetSpeedTPS);
        launcherRight.setVelocity(targetSpeedTPS);
    }

    public void stopSpin() {
        spin(STOP_POWER);
    }

    public void fire() {
        feeder.setPower(1);
    }

    public void resetFeeder() {
        feeder.setPower(0);
    }
    public double[] getWheelPRMs(){
        return new double[]{getLeftWheelRPM(), getRightWheelRPM()};
    }
    public static double RRP = 145.1; // for 5.1 : 1 1150 RPM
    public double getLeftWheelRPM() {
        // get velocity returns TPS, ticks per second
        return launcherLeft.getVelocity() / RRP * 60;
    }

    public double getRightWheelRPM() {
        return launcherRight.getVelocity() / RRP * 60;
    }

    // How long the wheels must have been driving forward before the feeder runs.
    // Set to 0 to feed immediately.
    public static double SPIN_UP_TIME = 0.4; // seconds

    public void manualLaunch(LauncherControls controls) {
        // debug the relation between ticks and angle, remove later
        telemetry.addData("RPM-left", this.getLeftWheelRPM());
        telemetry.addData("RPM-right", this.getRightWheelRPM());

        launcherLeft.setPower(controls.leftWheelPower);
        launcherRight.setPower(controls.rightWheelPower);

        // Restart the clock whenever the wheels are not driving forward, so the
        // timer always reads "how long have we been spinning up".
        if (controls.leftWheelPower <= 0) {
            timer.reset();
        }
        boolean upToSpeed = timer.seconds() >= SPIN_UP_TIME;

        if (controls.triggerDown && upToSpeed) {
            fire();
            telemetry.addData("Launcher", "FIRING");
        } else {
            resetFeeder();
            telemetry.addData("Launcher", controls.triggerDown ? "spinning up" : "idle");
        }
    }

    public void abort() {
        stopSpin();
        resetFeeder();
        launchState = LaunchState.IDLE;
    }
}

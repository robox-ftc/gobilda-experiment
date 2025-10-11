package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import static org.firstinspires.ftc.teamcode.Utils.applyAction;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Launcher {

    public static double LAUNCHER_MAX_VELOCITY = 1620;
    public static double LAUNCHER_MIN_VELOCITY = 1075;

    public static double FEEDER_ANGLE_SPAN = 300; // for goblida 2000-2500-0002
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;

    private double feederFireAngle = 0.0;
    private double feederReloadAngle = 0.0;

    public static double FEED_TIME_SECONDS = 0.20; //The feeder servos run this long when a shot is requested.
    public enum BulletColor
    {
        Green,
        Purple
    }

    public enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
        RELOAD,
        // TODO: add more states, e.g.
        ABORTING
    }

    public LaunchState launchState = LaunchState.IDLE;
    private DcMotorEx launcherLeft = null;
    private DcMotorEx launcherRight = null;
    private DcMotorEx[] launchers = new DcMotorEx[2];
    private Servo feeder = null;
    public boolean shootRequested = false;

    public double targetSpeed = 0.0;
    public double targetPower = 0.0;
    ElapsedTime feederTimer = new ElapsedTime();
    Telemetry telemetry;

    public Launcher(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        initShooter(hardwareMap);
        initFeeder(hardwareMap);
    }
    public void initShooter(HardwareMap hardwareMap) {
        launcherLeft = hardwareMap.get(DcMotorEx.class, "left_launcher");
        launcherRight = hardwareMap.get(DcMotorEx.class, "right_launcher");

        launchers[0] = launcherLeft;
        launchers[1] = launcherRight;
        launcherLeft.setDirection(DcMotor.Direction.REVERSE);
        launcherRight.setDirection(DcMotor.Direction.FORWARD);

        // for now, to be changed
        applyAction(launchers, (motor) -> motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER));
        applyAction(launchers, (motor) -> motor.setZeroPowerBehavior(FLOAT));
        applyAction(launchers, (motor) -> motor.setPower(0.0));
    }

    public void initFeeder(HardwareMap hardwareMap){
        /*
         * set Feeders to an initial value to initialize the servo controller
         */
        telemetry.addLine("init feeder");
        feeder = hardwareMap.get(Servo.class, "feeder");

        // The following setting depends on your hardware mountings.
        feeder.resetDeviceConfigurationForOpMode();
        feeder.setPosition(0.5);
        telemetry.addLine("init position" + feeder.getPosition());
        this.feederReloadAngle = 0.5;
        this.feederFireAngle = this.feederReloadAngle - 75.0 / FEEDER_ANGLE_SPAN;
        telemetry.addLine("range=" + this.feederReloadAngle + ", " + this.feederFireAngle);

        feederTimer = new ElapsedTime();
    }

    public void setTargetPower(double power){
        this.targetPower = power;
    }

    public void spin(double power){
       launcherLeft.setPower(power);
       launcherRight.setPower(power);
    }

    public void stopSpin(){
        launcherLeft.setPower(STOP_SPEED);
        launcherRight.setPower(STOP_SPEED);
    }

    public void fire(){
        feeder.setPosition(this.feederFireAngle);
    }

    public void reload(){
        feeder.setPosition(this.feederReloadAngle);
    }

    public double getFeederAngle(){
        return feeder.getPosition();
    }

    public void manualLaunch(LauncherControls controls) {
        spin(controls.wheelPower);
        if (controls.trigger)
            fire();
        else
            reload();
    }

    public void autoRun(){
        launch(this.shootRequested);
    }

    public void launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                spin(this.targetPower);
                if (launcherLeft.getVelocity() >= this.targetSpeed) {
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                fire();
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    reload();
                    stopSpin();
                    launchState = LaunchState.RELOAD;
                    feederTimer.reset();
                }
                break;
            case RELOAD:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    launchState = LaunchState.IDLE;
                }
        }
    }
}

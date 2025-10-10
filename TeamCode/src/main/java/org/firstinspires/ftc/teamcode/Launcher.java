package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import static org.firstinspires.ftc.teamcode.Utils.applyAction;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Launcher {

    public static double LAUNCHER_MAX_VELOCITY = 1620;
    public static double LAUNCHER_MIN_VELOCITY = 1075;
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;

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
        // TODO: add more states
        ABORTING
    }

    public LaunchState launchState = LaunchState.IDLE;
    public LaunchState manualLaunchState = LaunchState.IDLE;

    private DcMotorEx launcherLeft = null;
    private DcMotorEx launcherRight = null;
    private DcMotorEx[] launchers = new DcMotorEx[2];
    private CRServo feeder = null;

    public boolean autoMode = false;
    public boolean shootRequested = false;
    public double launchWheelSpeed = 0.0;
    public double targetSpeed = 0.0;
    public double targetPower = 0.0;
    ElapsedTime feederTimer = new ElapsedTime();

    private LauncherControls launcherControls = new LauncherControls();

    public Launcher(HardwareMap hardwareMap){
        initShooter(hardwareMap);
        initFeeder(hardwareMap);
    }
    public void initShooter(HardwareMap hardwareMap) {
        launcherLeft = hardwareMap.get(DcMotorEx.class, "left-launcher");
        launcherRight = hardwareMap.get(DcMotorEx.class, "right-launcher");

        launchers[0] = launcherLeft;
        launchers[1] = launcherRight;
        launcherLeft.setDirection(DcMotor.Direction.REVERSE);
        launcherRight.setDirection(DcMotor.Direction.FORWARD);

        // for now, to be changed
        applyAction(launchers, (motor) -> motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER));
        applyAction(launchers, (motor) -> motor.setZeroPowerBehavior(BRAKE));
        applyAction(launchers, (motor) -> motor.setPower(0.0));
    }

    public void initFeeder(HardwareMap hardwareMap){
        /*
         * set Feeders to an initial value to initialize the servo controller
         */

        feeder = hardwareMap.get(CRServo.class, "feeder");
        feeder.setPower(STOP_SPEED);
        feederTimer = new ElapsedTime();
    }

    public void update(LauncherControls newControls){
        if (this.launcherControls.modeButton && !newControls.modeButton)
        {
            autoMode = !autoMode;
        }

        if (this.launcherControls.trigger && !newControls.trigger)
        {
            shootRequested = true;
        }

        if (autoMode) {
            this.targetSpeed = STOP_SPEED; // not implemented yet.
            this.targetPower = 0.0; // then calculate according to targetSpeed.
        }
        else {
            this.targetPower = newControls.wheelPower;
        }

        // replace the launcher controls with the new ones
        this.launcherControls.modeButton = newControls.modeButton;
        this.launcherControls.trigger = newControls.trigger;
        this.launcherControls.wheelPower = newControls.wheelPower;
    }

    public void spin(){
       launcherLeft.setPower(this.targetPower);
    }

    public void shoot(){
        feeder.setPower(FULL_SPEED);
        feederTimer.reset();
    }

    public void reload(){
        feeder.setPower(-FULL_SPEED);
        feederTimer.reset();
    }


    public void manualLaunch() {
        spin();
        switch (manualLaunchState) {
            case IDLE:
                if (this.shootRequested) {
                    shoot();
                }
                manualLaunchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    reload();
                    launchState = LaunchState.RELOAD;
                }
                break;
            case RELOAD:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    launchState = LaunchState.IDLE;
                }
        }


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
                spin();
                if (launcherLeft.getVelocity() >= this.targetSpeed) {
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                shoot();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    reload();
                    launchState = LaunchState.RELOAD;
                }
                break;
            case RELOAD:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    launchState = LaunchState.IDLE;
                }
        }
    }
}

package org.firstinspires.ftc.teamcode.devices;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import org.firstinspires.ftc.teamcode.utils.Utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Launcher {
    public static double LAUNCHER_TICKS_PER_REV = 103.8; // for 1620 RPM motor
    public static double LAUNCHER_MAX_VELOCITY_RPM = 1620;
    public static double LAUNCHER_MAX_VELOCITY_DPS = LAUNCHER_MAX_VELOCITY_RPM / 60.0;
    public static double ERROR_RATE = 0.1;
    final double STOP_POWER = 0.0; // We send this power to the servos when we want them to stop.

    private DcMotorEx Launcher = null;
    ElapsedTime timer = new ElapsedTime();
    Telemetry telemetry;
    public Launcher(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        initShooter(hardwareMap);
    }

    public void initShooter(HardwareMap hardwareMap) {
        Launcher = hardwareMap.get(DcMotorEx.class, "Launcher");
        DcMotorEx[] launchers = new DcMotorEx[1];
        launchers[0] = Launcher;
        Launcher.setDirection(DcMotor.Direction.FORWARD);
        // for now, to be changed
        Utils.applyAction(launchers, (motor) -> motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER));
        Utils.applyAction(launchers, (motor) -> motor.setZeroPowerBehavior(FLOAT));
        Utils.applyAction(launchers, (motor) -> motor.setPower(0.0));
    }

    public void run(LauncherControls controls)
    {
        manualLaunch(controls);
    }

    public void spin(double power) {
        Launcher.setPower(power);
    }

    public void spinToVelocity(double targetSpeedTPS) { // ticks per second
        Launcher.setVelocity(targetSpeedTPS);
    }

    public void stopSpin() {
        spin(STOP_POWER);
    }
    public double[] getWheelPRMs(){
        return new double[]{getLauncherRPM()};
    }
    public static double RRP = 145.1; // for 5.1 : 1 1150 RPM
    public double getLauncherRPM() {
        // get velocity returns TPS, ticks per second
        return Launcher.getVelocity() / RRP * 60;
    }

    // How long the wheels must have been driving forward before the feeder runs.
    // Set to 0 to feed immediately.
    public static double SPIN_UP_TIME = 0.4; // seconds

    public void manualLaunch(LauncherControls controls) {
        // debug the relation between ticks and angle, remove later
        telemetry.addData("RPM-Launcher", this.getLauncherRPM());

        Launcher.setPower(controls.LauncherPower);

        // Restart the clock whenever the wheels are not driving forward, so the
        // timer always reads "how long have we been spinning up".
        if (controls.LauncherPower <= 0) {
            timer.reset();
        }
        boolean upToSpeed = timer.seconds() >= SPIN_UP_TIME;
    }
}

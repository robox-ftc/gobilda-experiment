package org.firstinspires.ftc.teamcode.devices;

public class LauncherControls
{
    public double LauncherPower;
    public boolean bumperDown;
    public boolean leftTrigger;
    public boolean rightTrigger;

    public LauncherControls() {
        this.LauncherPower = 0.0;
        this.bumperDown = false;
    }

    public LauncherControls(double wheelPower, boolean triggerDown) {
        this.LauncherPower = wheelPower;
        this.bumperDown = triggerDown;
    }

    public static double computeTurretRotationPower(double currentAngle, double targetAngle, double tolerance){
        double diff = targetAngle - currentAngle;
        return Math.abs(diff) < tolerance ? 0 : Math.signum(diff); // Always full power; no PID
    }


    // The motor max speed is 1033 with wheels installed per experiments.
    private static double kP = 0.012;   // ← Start here

    public static double[] computeShooterWheelPower(double[] rpms, double targetRPMs){
        double errorLauncher = targetRPMs - rpms[0];
        double powerLauncher = errorLauncher * kP;
        return new double[]{ Math.max(0, Math.min(1.0, powerLauncher))};
    }


    // Wheel power used when the driver asks to fire without having spun up first.
    public static final double DEFAULT_LAUNCH_POWER = 0.75;

    public static LauncherControls readControls(GamePadReadings newReadings) {
        LauncherControls controls = new LauncherControls();
        controls.LauncherPower = newReadings.leftTrigger;
        if (newReadings.aButton) {
            controls.LauncherPower = 0.75;
        }
        if (newReadings.yButton) {
            controls.LauncherPower = 1;
        }
        if (newReadings.bButton) {
            controls.LauncherPower = -0.75;
        }
        controls.bumperDown = newReadings.leftBumper || newReadings.rightBumper;
        controls.leftTrigger =  newReadings.leftBumper;
        controls.rightTrigger = newReadings.rightBumper;

        // Feeding a ball into stopped wheels just jams it, so a bare fire press
        // spins the wheels up too. B (eject) keeps its negative power.
        if (controls.bumperDown && controls.LauncherPower == 0) {
            controls.LauncherPower = DEFAULT_LAUNCH_POWER;
        }
        return controls;
    }
}

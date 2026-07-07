package org.firstinspires.ftc.teamcode.devices;

import org.firstinspires.ftc.teamcode.data.AimingParameters;
import org.firstinspires.ftc.teamcode.data.Target;

public class LauncherControls
{
    public double leftWheelPower;
    public double rightWheelPower;
    public boolean triggerDown;

    public LauncherControls() {
        this.leftWheelPower = 0.0;
        this.rightWheelPower = 0.0;
        this.triggerDown = false;
    }

    public LauncherControls(double wheelPower, boolean triggerDown) {
        this.leftWheelPower = wheelPower;
        this.rightWheelPower = wheelPower;
        this.triggerDown = triggerDown;
    }


    public LauncherControls(double leftWheelPower, double rightWheelPower, boolean triggerDown)
    {
        this.leftWheelPower = leftWheelPower;
        this.rightWheelPower = rightWheelPower;
        this.triggerDown = triggerDown;
    }

    public static double computeTurretRotationPower(double currentAngle, double targetAngle, double tolerance){
        double diff = targetAngle - currentAngle;
        return Math.abs(diff) < tolerance ? 0 : Math.signum(diff); // Always full power; no PID
    }


    // The motor max speed is 1033 with wheels installed per experiments.
    private static double kP = 0.012;   // ← Start here

    public static double[] computeShooterWheelPower(double[] rpms, double targetRPMs){
        double errorLeft = targetRPMs - rpms[0];
        double errorRight = targetRPMs - rpms[1];
        double powerLeft = errorLeft * kP;
        double powerRight = errorRight * kP;
        return new double[]{ Math.max(0, Math.min(1.0, powerLeft)), Math.max(0, Math.min(1.0, powerRight))};
    }


    public static LauncherControls readControls(GamePadReadings newReadings) {
        LauncherControls controls = new LauncherControls();
        controls.leftWheelPower = newReadings.leftTrigger;
        controls.rightWheelPower = newReadings.leftTrigger;
        if (newReadings.aWasReleased) {

        }
        controls.triggerDown = newReadings.leftBumper;
        return controls;
    }
}

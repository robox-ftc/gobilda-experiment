package org.firstinspires.ftc.teamcode.devices;

import org.firstinspires.ftc.teamcode.data.AimingParameters;
import org.firstinspires.ftc.teamcode.data.Target;

public class LauncherControls
{
    public double leftWheelPower;
    public double rightWheelPower;
    public double turretPower;
    public boolean triggerDown;

    public LauncherControls(){
        this.leftWheelPower = 0.0;
        this.rightWheelPower = 0.0;
        this.turretPower = 0.0;
        this.triggerDown = false;
    }

    public LauncherControls(double wheelPower, double turretPower, boolean triggerDown){
        this.leftWheelPower = wheelPower;
        this.rightWheelPower = wheelPower;
        this.turretPower = turretPower;
        this.triggerDown = triggerDown;
    }


    public LauncherControls(double leftWheelPower, double rightWheelPower, double turretPower, boolean triggerDown)
    {
        this.leftWheelPower = leftWheelPower;
        this.rightWheelPower = rightWheelPower;
        this.turretPower = turretPower;
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

    public static AimingParameters computeAimingParameters(Target target)
    {
        // An experiment shows the max rmp reading is about 1033.
        // TODO: collect data, make a lookup table, the reading is not accurate, UNIT is inch
        if (target.distance/12.0 > 8) { //ft, shoot from farther zone
            return new AimingParameters(30.0, 1000.0);
        } else if (target.distance <= 8){
            return new AimingParameters(60.0, 700.0);
        }

        return new AimingParameters();
    }

    public static LauncherControls readControls(GamePadReadings newReadings) {
        LauncherControls controls = new LauncherControls();
        controls.leftWheelPower = newReadings.leftTrigger;
        controls.rightWheelPower = newReadings.leftTrigger;
        controls.triggerDown = newReadings.aButton;
        // constrain angle from 0 to 1, we want it only goes to one position above the "zero" point.
        controls.turretPower = (newReadings.dPadUp ? 1.0 : 0.0) - (newReadings.dPadDown ? 1.0 : 0.0);
        return controls;
    }
}

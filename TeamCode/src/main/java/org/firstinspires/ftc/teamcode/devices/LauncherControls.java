package org.firstinspires.ftc.teamcode.devices;

import org.firstinspires.ftc.teamcode.data.AimingParameters;
import org.firstinspires.ftc.teamcode.data.Target;

public class LauncherControls {
    public double wheelPower;
    public double rampPower;

    public LauncherControls() {
        this.wheelPower = 0.0;
        this.rampPower = 0.0;
    }

    public LauncherControls(double wheelPower, double triggerDown) {
        this.wheelPower = wheelPower;
        this.rampPower = 0.0;
    }


    // The motor max speed is 1033 with wheels installed per experiments.
    private static double kP = 0.012;   // ← Start here


    public static LauncherControls readControls(GamePadReadings newReadings) {
        LauncherControls controls = new LauncherControls();
        controls.wheelPower = newReadings.leftTrigger;
        if (newReadings.bButton) {
            controls.wheelPower = -0.75;
        }
        controls.rampPower = newReadings.bButton ? -1 : ((newReadings.leftBumper || newReadings.rightBumper) ? 1 : 0);
        if (newReadings.aButton) {
            controls.rampPower = -controls.rampPower;
        }
        return controls;
    }
}

package org.firstinspires.ftc.teamcode.devices;

import org.firstinspires.ftc.teamcode.TeleOp;
import org.firstinspires.ftc.teamcode.data.AimingParameters;
import org.firstinspires.ftc.teamcode.data.Target;

public class LauncherControls {
    public static final double TARGET_FRACTION = 0.6;
    public double wheelPower;
    public double rampPower;
    public double intake;

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
        if (newReadings.rightBumper) {
            controls.rampPower = 1;
            controls.intake = 1;
        } else if (newReadings.bButton) {
            controls.wheelPower = -0.5;
            controls.rampPower = -0.7;
            controls.intake = -0.7;
        } else {
            controls.wheelPower *= TARGET_FRACTION;
        }
        if (newReadings.aButton) {
            controls.rampPower = 1;
        }
        return controls;
    }
}

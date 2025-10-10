package org.firstinspires.ftc.teamcode;

public class LauncherControls
{
    public double wheelPower;
    public boolean trigger;
    public boolean modeButton;

    public LauncherControls(){
        this.wheelPower = 0;
        this.trigger = false;
        this.modeButton = false;
    }

    public LauncherControls(double wheelPress, boolean trigger, boolean modeButton) {
        this.wheelPower = wheelPress;
        this.trigger = trigger;
        this.modeButton = modeButton;
    }
}

package org.firstinspires.ftc.teamcode;

public class LauncherControls
{
    public double wheelPower;
    public boolean trigger;

    public LauncherControls(){
        this.wheelPower = 0;
        this.trigger = false;
    }

    public LauncherControls(double wheelPress, boolean trigger) {
        this.wheelPower = wheelPress;
        this.trigger = trigger;
    }
}

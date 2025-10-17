package org.firstinspires.ftc.teamcode.Utils;

public class LauncherControls
{
    public double wheelPower;
    public boolean trigger;

    public LauncherControls(){
        this.wheelPower = 0;
    }

    public LauncherControls(double wheelPress) {
        this.wheelPower = wheelPress;
    }
}

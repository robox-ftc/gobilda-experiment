package org.firstinspires.ftc.teamcode.Utils;

public class LauncherControls
{
    public double wheelPower;
    public boolean fireRequested;
    public boolean triggerDown;
    public double turretPower;
    public boolean abortRequested;

    public LauncherControls(){
        this.wheelPower = 0;
        this.fireRequested = false;
        this.triggerDown = false;
        this.turretPower = 0.0;
        this.abortRequested = false;
    }
}

package org.firstinspires.ftc.teamcode.Utils;

public class LauncherControls
{
    public double wheelPower;
    public boolean fireRequested;
    public boolean triggerDown;
    public double turretAngle;
    public boolean abortRequested;

    public LauncherControls(){
        this.wheelPower = 0;
        this.fireRequested = false;
        this.triggerDown = false;
        this.turretAngle = 0.0;
        this.abortRequested = false;
    }
}

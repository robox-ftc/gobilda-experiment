package org.firstinspires.ftc.teamcode.data;

public class AimingParameters {
    public Double shootingAngle; // unit degree
    public Double launcherRpm;  // unit PRM

    public AimingParameters(){
        this.shootingAngle = null;
        this.launcherRpm = null;
    }
    public AimingParameters(Double turretAngle, Double launcherRpm){
        this.shootingAngle = turretAngle;
        this.launcherRpm = launcherRpm;
    }
}

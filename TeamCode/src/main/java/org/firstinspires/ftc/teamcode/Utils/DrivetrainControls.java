package org.firstinspires.ftc.teamcode.Utils;

public class DrivetrainControls {
    public double translationXPower;
    public double translationYPower;
    public double rotationPower;

    public String toString(){
        return "dr x=" + translationXPower + ",y=" + translationYPower +",r="+rotationPower;
    }
    public DrivetrainControls(double translationX, double translationY, double rotation) {
        this.translationXPower = translationX;
        this.translationYPower = translationY;
        this.rotationPower = rotation;
    }

    public DrivetrainControls() {
        this.translationXPower = 0;
        this.translationYPower = 0;
        this.rotationPower = 0;
    }
}

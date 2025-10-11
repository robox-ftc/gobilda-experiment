package org.firstinspires.ftc.teamcode;

public class DrivetrainControls {
    public double translationX;
    public double translationY;
    public double rotation;

    public DrivetrainControls(double translationX, double translationY, double rotation) {
        this.translationX = translationX;
        this.translationY = translationY;
        this.rotation = rotation;
    }

    public DrivetrainControls(){
        this.translationX = 0;
        this.translationY = 0;
        this.rotation = 0;
    }
}

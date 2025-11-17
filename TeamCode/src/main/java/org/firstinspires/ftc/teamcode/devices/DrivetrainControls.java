package org.firstinspires.ftc.teamcode.devices;

import org.firstinspires.ftc.teamcode.data.Vec2;

public class DrivetrainControls {
    public double translationXPower;
    public double translationYPower;
    public double rotationPower;

    public String toString(){
        return "dr-pwr x=" + translationXPower + ",y=" + translationYPower +",r="+rotationPower;
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


    private static double AimingKp = 0.075;
    private static double MovingKp = 0.75;

    public static double computeAimingPower(double angle, double tolerance){
        double rotatePower = 0;
        if (Math.abs(angle) > tolerance )
            rotatePower = AimingKp * angle; // notice the sign.
        return rotatePower;
    }

    public static DrivetrainControls computeTranslationPower(Vec2 v, Vec2 tolerance){
        return new DrivetrainControls(){{
            translationXPower = Math.abs(v.x) > tolerance.x ? MovingKp * v.x : 0.0;
            translationYPower = Math.abs(v.y) > tolerance.y ? MovingKp * v.y : 0.0;
            rotationPower = 0.0;
        }};
    }
    public static DrivetrainControls readControls(GamePadReadings newReadings)
    {
        return new DrivetrainControls(){{
            translationXPower = newReadings.leftStickX;
            translationYPower = -newReadings.leftStickY;
            rotationPower = newReadings.rightStickX;
        }};
    }

    public void combinePower(double translationXPower, double translationYPower, double rotationPower){
        this.translationXPower += translationXPower;
        this.translationYPower += translationYPower;
        this.rotationPower += rotationPower;
    }

    public void combineControls(DrivetrainControls other){
        this.translationXPower += other.translationXPower;
        this.translationYPower += other.translationYPower;
        this.rotationPower += other.rotationPower;
    }
}

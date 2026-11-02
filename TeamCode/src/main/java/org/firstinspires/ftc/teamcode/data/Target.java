package org.firstinspires.ftc.teamcode.data;

public class Target {
    public double angle;
    public double heading;

    public Target(double a, double d){
        this.angle = a;
        this.heading = d;
    }
    public Target(){
        this.angle = 0;
        this.heading = 0;
    }

    public String toString(){
        return String.format("target at (a=%.4f,d=%.4f)", angle, heading);
    }
}

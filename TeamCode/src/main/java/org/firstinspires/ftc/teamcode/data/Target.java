package org.firstinspires.ftc.teamcode.data;

public class Target {
    public double angle;
    public double distance;

    public Target(double a, double d){
        this.angle = a;
        this.distance = d;
    }
    public Target(){
        this.angle = 0;
        this.distance = 0;
    }

    public String toString(){
        return String.format("target at (a=%.4f,d=%.4f)", angle, distance);
    }
}

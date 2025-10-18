package org.firstinspires.ftc.teamcode.Utils;

public class Task {

    public static final int TRANSLATE = 0;
    public static final int ROTATE = 1;
    public static final int LAUNCH = 2;
    public int begin;
    public int end;
    public double magnitude;
    public int type;

    public Task() {}
    public Task(double magnitude, int type) {
        this.magnitude = magnitude;
        this.type = type;
    }
    public Task(int begin, int end, double magnitude, int type) {
        this.begin = begin;
        this.end = end;
        this.magnitude = magnitude;
        this.type = type;
    }
}

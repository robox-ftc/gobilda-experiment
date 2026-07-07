package org.firstinspires.ftc.teamcode.devices;

public class Task {

    public static final int TRANSLATE = 0;
    public static final int ROTATE = 1;
    public static final int LAUNCH = 2;
    public int begin;
    public int end;
    public boolean executed;
    public int type;
    public double target;
    public double target2;

    public Task() {}
    public Task(int target, int type) {
        this.type = type;
        this.target = target;
        this.executed = false;
    }
    public Task(int begin, int end, int type, double target) {
        this.begin = begin;
        this.end = end;
        this.type = type;
        this.target = target;
        this.executed = false;
    }
    public Task(int begin, int end, int type, double x, double y) {
        this.begin = begin;
        this.end = end;
        this.type = type;
        this.target = x;
        this.target2 = y;
        this.executed = false;
    }
}

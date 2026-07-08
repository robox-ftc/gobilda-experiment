package org.firstinspires.ftc.teamcode.devices;

public class Task {
    public enum Type {
        TRANSLATE,
        ROTATE,
        LAUNCH
    }
    public int begin;
    public int end;
    public boolean executed;
    public Type type;
    public double target;
    public double target2;

    public Task() {}
    public Task(int target, Type type) {
        this.type = type;
        this.target = target;
    }
    public Task(int begin, int end, Type type, double target) {
        this.begin = begin;
        this.end = end;
        this.type = type;
        this.target = target;
    }
    public Task(int begin, int end, Type type, double x, double y) {
        this.begin = begin;
        this.end = end;
        this.type = type;
        this.target = x;
        this.target2 = y;
    }
}

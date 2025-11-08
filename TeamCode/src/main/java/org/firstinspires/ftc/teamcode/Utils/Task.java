package org.firstinspires.ftc.teamcode.Utils;

public class Task {

    public static final int TRANSLATE = 0;
    public static final int ROTATE = 1;
    public static final int LAUNCH = 2;
    public int begin;
    public int end;
    public int type;
    public int target;

    public Task() {}
    public Task(int target, int type) {
        this.type = type;
        this.target = target;
    }
    public Task(int begin, int end, int type, int target) {
        this.begin = begin;
        this.end = end;
        this.type = type;
        this.target = target;
    }
}

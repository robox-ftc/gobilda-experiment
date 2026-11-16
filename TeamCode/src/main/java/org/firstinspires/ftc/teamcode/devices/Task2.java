package org.firstinspires.ftc.teamcode.devices;

public class Task2 { //Used for BIOBUZZAutoOdo
    public enum Type {
        TRANSLATE,
        ROTATE,
        LAUNCH,
        INTAKE
    }
    public int end;
    public boolean executed;
    public Type type;
    public double FieldX;
    public double FieldY;
    public double Heading;
    public double SpinUpTime;
    public double LauncherPower;

    public Task2() {}
    public Task2(int target, Type type) {
        this.type = type;
        this.FieldX = FieldX;
    }
    public Task2(int end, Type type, double Heading) {
        this.end = end;
        this.type = type;
        this.Heading = Heading;
    }
    public Task2(int end, Type type, double FieldX, double FieldY) {
        this.end = end;
        this.type = type;
        this.FieldX = FieldX;
        this.FieldY = FieldY;
    }
}

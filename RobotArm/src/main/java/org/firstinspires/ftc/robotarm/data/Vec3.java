package org.firstinspires.ftc.robotarm.data;

public class Vec3{
    public double x;
    public double y;
    public double z;

    public Vec3() {
        this.x = 0;
        this.y = 0;
        this.z = 0;
    }

    public Vec3(double[] coords) {
        this.x = coords[0];
        this.y = coords[1];
        this.z = coords[2];
    }

    public Vec3(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public Vec3 add(Vec3 other) {
        return new Vec3(this.x + other.x, this.y + other.y, this.z + other.z);
    }

    public Vec3 sub(Vec3 other) {
        return new Vec3(this.x - other.x, this.y - other.y, this.z - other.z);
    }

    public Vec3 scale(double scalar) {
        return new Vec3(this.x * scalar, this.y * scalar, this.z * scalar);
    }

    public double dot(Vec3 other) {
        return this.x * other.x + this.y * other.y + this.z * other.z;
    }

    public double norm() {
        return Math.sqrt(this.dot(this));
    }

    public Vec3 normalize() {
        double norm = this.norm();
        return this.scale(1.0 / norm);
    }
    
    public Vec3 cross(Vec3 other) {
        return new Vec3(this.y * other.z - this.z * other.y, this.z * other.x - this.x * other.z, this.x * other.y - this.y * other.x);
    }

    public static Vec3 zero() {
        return new Vec3(0.0, 0.0, 0.0);
    }

    public static Vec3 origin() {
        return new Vec3(0.0, 0.0, 0.0);
    }

    public static Vec3 unitX() {
        return new Vec3(1.0, 0.0, 0.0);
    }

    public static Vec3 unitY() {
        return new Vec3(0.0, 1.0, 0.0);
    }

    public static Vec3 unitZ() {
        return new Vec3(0.0, 0.0, 1.0);
    }
}

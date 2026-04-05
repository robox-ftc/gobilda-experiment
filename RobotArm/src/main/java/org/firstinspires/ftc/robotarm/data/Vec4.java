package org.firstinspires.ftc.robotarm.data;

public class Vec4 {
    public double x;
    public double y;
    public double z;
    public double w;
    public Vec4(double x, double y, double z, double w) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = w;
    }

    public Vec4() {
        this(0.0, 0.0, 0.0, 0.0);
    }
    public Vec4(Vec4 other) {
        this(other.x, other.y, other.z, other.w);
    }
    public Vec4 add(Vec4 other) {
        return new Vec4(this.x + other.x, this.y + other.y, this.z + other.z, this.w + other.w);
    }

    public Vec4 sub(Vec4 other) {
        return new Vec4(this.x - other.x, this.y - other.y, this.z - other.z, this.w - other.w);
    }

    public Vec4 scale(double scalar) {
        return new Vec4(this.x * scalar, this.y * scalar, this.z * scalar, this.w * scalar);
    }

    public double dot(Vec4 other) {
        return this.x * other.x + this.y * other.y + this.z * other.z + this.w * other.w;
    }

    public double norm() {
        return Math.sqrt(this.dot(this));
    }

    public Vec4 normalize() {
        double norm = this.norm();
        return this.scale(1.0/norm);
    }
    public Vec4 cross(Vec4 other) {
        return new Vec4(this.y * other.z - this.z * other.y, this.z * other.x - this.x * other.z, this.x * other.y - this.y * other.x, 0.0);
    }

    public static Vec4 zero() {
        return new Vec4(0.0, 0.0, 0.0, 0.0);
    }

    public static Vec4 origin() {
        return new Vec4(0.0, 0.0, 0.0, 1.0);
    }

    public static Vec4 unitX() {
        return new Vec4(1.0, 0.0, 0.0, 0.0);
    }

    public static Vec4 unitY() {
        return new Vec4(0.0, 1.0, 0.0, 0.0);
    }

    public static Vec4 unitZ() {
        return new Vec4(0.0, 0.0, 1.0, 0.0);
    }

    public static Vec4 unitW() {
        return new Vec4(0.0, 0.0, 0.0, 1.0);
    }

    public static Vec4 homogeneous(double x, double y, double z) {
        return new Vec4(x, y, z, 1.0);
    }

    public static Vec4 homogeneous(double x, double y, double z, double w) {
        return new Vec4(x, y, z, w);
    }

    public static Vec4 homogeneous(Vec3 vec) {
        return new Vec4(vec.x, vec.y, vec.z, 1.0);
    }
}


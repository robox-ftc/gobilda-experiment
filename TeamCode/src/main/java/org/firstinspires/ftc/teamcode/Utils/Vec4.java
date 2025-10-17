package org.firstinspires.ftc.teamcode.Utils;

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

    public Vec4 mul(double scalar) {
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
        return this.mul(1.0/norm);
    }
}

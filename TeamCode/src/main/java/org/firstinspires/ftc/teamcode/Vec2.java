package org.firstinspires.ftc.teamcode;

public class Vec2{
    public double x;
    public double y;

    public Vec2() {
        this.x = 0;
        this.y = 0;
    }

    public Vec2(double[] coords) {
        this.x = coords[0];
        this.y = coords[1];
    }
    public Vec2(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vec2 add(Vec2 other) {
        return new Vec2(this.x + other.x, this.y + other.y);
    }

    public Vec2 sub(Vec2 other) {
        return new Vec2(this.x - other.x, this.y - other.y);
    }

    public Vec2 scale(double scalar) {
        return new Vec2(this.x * scalar, this.y * scalar);
    }

    public double dot(Vec2 other) {
        return this.x * other.x + this.y * other.y;
    }

    public double norm(){
        return Math.sqrt(this.dot(this));
    }

    public Vec2 normalize() {
        double norm = this.norm();
        return this.scale(1.0 / norm);
    }
}

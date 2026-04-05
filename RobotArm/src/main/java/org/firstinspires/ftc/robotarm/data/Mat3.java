package org.firstinspires.ftc.robotarm.data;

/**
 * 3 × 3 matrix composed of three row vectors.
 * Used for rotation matrices, inertia tensors, etc.
 */
public class Mat3 {
    public Vec3 r0;
    public Vec3 r1;
    public Vec3 r2;

    public Mat3() {
        this.r0 = new Vec3();
        this.r1 = new Vec3();
        this.r2 = new Vec3();
    }

    public Mat3(Vec3 r0, Vec3 r1, Vec3 r2) {
        this.r0 = r0;
        this.r1 = r1;
        this.r2 = r2;
    }

    public Mat3(double[] m) {
        this.r0 = new Vec3(m[0], m[1], m[2]);
        this.r1 = new Vec3(m[3], m[4], m[5]);
        this.r2 = new Vec3(m[6], m[7], m[8]);
    }

    public static Mat3 identity() {
        return new Mat3(
            new Vec3(1, 0, 0),
            new Vec3(0, 1, 0),
            new Vec3(0, 0, 1)
        );
    }

    public Mat3 inverse() {
        return new Mat3(
            new Vec3(r1.y * r2.z - r1.z * r2.y, r2.x * r0.y - r0.x * r2.y, r0.x * r1.y - r1.x * r0.y),
            new Vec3(r2.x * r0.z - r0.x * r2.z, r0.x * r1.z - r1.x * r0.z, r1.x * r0.y - r0.x * r1.y),
            new Vec3(r0.x * r1.z - r1.x * r0.z, r1.x * r2.y - r2.x * r1.y, r2.x * r0.y - r0.x * r2.y)
        );
    }

    public static Mat3 zero() {
        return new Mat3();
    }

    public Mat3 add(Mat3 other) {
        return new Mat3(
            this.r0.add(other.r0),
            this.r1.add(other.r1),
            this.r2.add(other.r2)
        );
    }

    public Mat3 sub(Mat3 other) {
        return new Mat3(
            this.r0.sub(other.r0),
            this.r1.sub(other.r1),
            this.r2.sub(other.r2)
        );
    }

    public Mat3 scale(double scalar) {
        return new Mat3(
            this.r0.scale(scalar),
            this.r1.scale(scalar),
            this.r2.scale(scalar)
        );
    }

    /** Matrix × vector. */
    public Vec3 mul(Vec3 v) {
        return new Vec3(r0.dot(v), r1.dot(v), r2.dot(v));
    }

    /** Matrix × matrix. */
    public Mat3 mul(Mat3 other) {
        Mat3 t = other.transpose();
        return new Mat3(
            new Vec3(r0.dot(t.r0), r0.dot(t.r1), r0.dot(t.r2)),
            new Vec3(r1.dot(t.r0), r1.dot(t.r1), r1.dot(t.r2)),
            new Vec3(r2.dot(t.r0), r2.dot(t.r1), r2.dot(t.r2))
        );
    }

    public Mat3 transpose() {
        return new Mat3(
            new Vec3(r0.x, r1.x, r2.x),
            new Vec3(r0.y, r1.y, r2.y),
            new Vec3(r0.z, r1.z, r2.z)
        );
    }

    public double determinant() {
        return r0.x * (r1.y * r2.z - r1.z * r2.y)
             - r0.y * (r1.x * r2.z - r1.z * r2.x)
             + r0.z * (r1.x * r2.y - r1.y * r2.x);
    }

    public double trace() {
        return r0.x + r1.y + r2.z;
    }

    @Override
    public String toString() {
        return String.format("[%s, %s, %s]", r0, r1, r2);
    }
}

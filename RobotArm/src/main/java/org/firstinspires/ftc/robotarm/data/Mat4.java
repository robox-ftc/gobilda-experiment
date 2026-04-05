package org.firstinspires.ftc.robotarm.data;

/**
 * 4 × 4 matrix composed of four row vectors.
 * Used for homogeneous transforms, combining rotation + translation
 * into a single matrix multiply.
 */
public class Mat4 {
    public Vec4 r0;
    public Vec4 r1;
    public Vec4 r2;
    public Vec4 r3;

    public Mat4() {
        this.r0 = new Vec4();
        this.r1 = new Vec4();
        this.r2 = new Vec4();
        this.r3 = new Vec4();
    }

    public Mat4(Vec4 r0, Vec4 r1, Vec4 r2, Vec4 r3) {
        this.r0 = r0;
        this.r1 = r1;
        this.r2 = r2;
        this.r3 = r3;
    }

    public Mat4(double[] m) {
        this.r0 = new Vec4(m[0],  m[1],  m[2],  m[3]);
        this.r1 = new Vec4(m[4],  m[5],  m[6],  m[7]);
        this.r2 = new Vec4(m[8],  m[9],  m[10], m[11]);
        this.r3 = new Vec4(m[12], m[13], m[14], m[15]);
    }

    public static Mat4 identity() {
        return new Mat4(
            new Vec4(1, 0, 0, 0),
            new Vec4(0, 1, 0, 0),
            new Vec4(0, 0, 1, 0),
            new Vec4(0, 0, 0, 1)
        );
    }

    public static Mat4 zero() {
        return new Mat4();
    }

    /**
     * Build a homogeneous transform from a 3×3 rotation and a translation.
     * Result:
     *   [ R  t ]
     *   [ 0  1 ]
     */
    public static Mat4 fromRotationTranslation(Mat3 rot, Vec3 t) {
        return new Mat4(
            new Vec4(rot.r0.x, rot.r0.y, rot.r0.z, t.x),
            new Vec4(rot.r1.x, rot.r1.y, rot.r1.z, t.y),
            new Vec4(rot.r2.x, rot.r2.y, rot.r2.z, t.z),
            new Vec4(0, 0, 0, 1)
        );
    }

    /** Pure translation, no rotation. */
    public static Mat4 fromTranslation(Vec3 t) {
        return fromRotationTranslation(Mat3.identity(), t);
    }

    /** Pure translation, no rotation. */
    public static Mat4 fromTranslation(double x, double y, double z) {
        return fromTranslation(new Vec3(x, y, z));
    }

    /** Extract the upper-left 3×3 rotation block. */
    public Mat3 rotation() {
        return new Mat3(
            new Vec3(r0.x, r0.y, r0.z),
            new Vec3(r1.x, r1.y, r1.z),
            new Vec3(r2.x, r2.y, r2.z)
        );
    }

    /** Extract the translation column (first three elements of column 3). */
    public Vec3 translation() {
        return new Vec3(r0.w, r1.w, r2.w);
    }

    public Mat4 add(Mat4 other) {
        return new Mat4(
            r0.add(other.r0),
            r1.add(other.r1),
            r2.add(other.r2),
            r3.add(other.r3)
        );
    }

    public Mat4 sub(Mat4 other) {
        return new Mat4(
            r0.sub(other.r0),
            r1.sub(other.r1),
            r2.sub(other.r2),
            r3.sub(other.r3)
        );
    }

    public Mat4 scale(double scalar) {
        return new Mat4(
            r0.scale(scalar),
            r1.scale(scalar),
            r2.scale(scalar),
            r3.scale(scalar)
        );
    }

    /** Matrix × vector. */
    public Vec4 mul(Vec4 v) {
        return new Vec4(r0.dot(v), r1.dot(v), r2.dot(v), r3.dot(v));
    }

    /**
     * Transform a 3D point (implicitly w = 1).
     * Returns the resulting 3D point after perspective divide.
     */
    public Vec3 transformPoint(Vec3 p) {
        Vec4 v = mul(new Vec4(p.x, p.y, p.z, 1));
        return new Vec3(v.x, v.y, v.z);
    }

    /**
     * Transform a 3D direction (implicitly w = 0).
     * Translation has no effect.
     */
    public Vec3 transformDirection(Vec3 d) {
        Vec4 v = mul(new Vec4(d.x, d.y, d.z, 0));
        return new Vec3(v.x, v.y, v.z);
    }

    /** Matrix × matrix. */
    public Mat4 mul(Mat4 other) {
        Mat4 t = other.transpose();
        return new Mat4(
            new Vec4(r0.dot(t.r0), r0.dot(t.r1), r0.dot(t.r2), r0.dot(t.r3)),
            new Vec4(r1.dot(t.r0), r1.dot(t.r1), r1.dot(t.r2), r1.dot(t.r3)),
            new Vec4(r2.dot(t.r0), r2.dot(t.r1), r2.dot(t.r2), r2.dot(t.r3)),
            new Vec4(r3.dot(t.r0), r3.dot(t.r1), r3.dot(t.r2), r3.dot(t.r3))
        );
    }

    public Mat4 transpose() {
        return new Mat4(
            new Vec4(r0.x, r1.x, r2.x, r3.x),
            new Vec4(r0.y, r1.y, r2.y, r3.y),
            new Vec4(r0.z, r1.z, r2.z, r3.z),
            new Vec4(r0.w, r1.w, r2.w, r3.w)
        );
    }

    /**
     * Fast inverse for a rigid-body (homogeneous) transform.
     * Assumes the matrix is [ R t ; 0 1 ] where R is orthonormal.
     * Result is [ Rᵀ  -Rᵀt ; 0  1 ].
     */
    public Mat4 inverseRigid() {
        Mat3 r = rotation();
        Mat3 rt = r.transpose();
        Vec3 t = translation();
        Vec3 invT = rt.mul(t).scale(-1);
        return fromRotationTranslation(rt, invT);
    }

    public double trace() {
        return r0.x + r1.y + r2.z + r3.w;
    }

    @Override
    public String toString() {
        return String.format("[%s, %s, %s, %s]", r0, r1, r2, r3);
    }
}

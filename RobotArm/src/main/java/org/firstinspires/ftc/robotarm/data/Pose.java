package org.firstinspires.ftc.robotarm.data;

/**
 * A 3D pose representing position and orientation in space.
 * Orientation is stored as Euler angles (roll, pitch, yaw) for student
 * accessibility. Sufficient for FTC arm kinematics; upgrade to quaternion
 * or rotation matrix only if gimbal-lock becomes a real problem.
 *
 * <p>Euler convention: <b>ZYX intrinsic</b> (yaw-pitch-roll).
 * The rotation matrix is built as R = Rz(rz) · Ry(ry) · Rx(rx).
 * This matches URDF's "rpy" attribute.
 */
public class Pose {
    public final Vec3 position;
    public final Vec3 rotation;   // Euler angles (rx, ry, rz) in radians

    public Pose(Vec3 position, Vec3 rotation) {
        this.position = position;
        this.rotation = rotation;
    }

    /** Identity pose: origin, no rotation. */
    public static Pose identity() {
        return new Pose(Vec3.zero(), Vec3.zero());
    }

    /**
     * Convert to a 4×4 homogeneous transform.
     * R = Rz(rz) · Ry(ry) · Rx(rx), then pack [R t; 0 1].
     */
    public Mat4 toMat4() {
        double rx = rotation.x, ry = rotation.y, rz = rotation.z;
        double cx = Math.cos(rx), sx = Math.sin(rx);
        double cy = Math.cos(ry), sy = Math.sin(ry);
        double cz = Math.cos(rz), sz = Math.sin(rz);

        Mat3 rot = new Mat3(
            new Vec3(cy * cz, sx * sy * cz - cx * sz, cx * sy * cz + sx * sz),
            new Vec3(cy * sz, sx * sy * sz + cx * cz, cx * sy * sz - sx * cz),
            new Vec3(-sy,     sx * cy,                 cx * cy)
        );
        return Mat4.fromRotationTranslation(rot, position);
    }

    /**
     * Extract a Pose from a homogeneous transform matrix.
     * Assumes the upper-left 3×3 is a proper rotation (det = +1).
     * Uses the ZYX Euler extraction: ry = asin(−R20), then
     * rx = atan2(R21, R22), rz = atan2(R10, R00).
     */
    public static Pose fromMat4(Mat4 m) {
        Vec3 t = m.translation();
        Mat3 r = m.rotation();

        double sy = -r.r2.x;
        sy = Math.max(-1.0, Math.min(1.0, sy));
        double ry = Math.asin(sy);

        double rx, rz;
        if (Math.abs(Math.cos(ry)) > 1e-9) {
            rx = Math.atan2(r.r2.y, r.r2.z);
            rz = Math.atan2(r.r1.x, r.r0.x);
        } else {
            // Gimbal lock: ry ≈ ±π/2
            rx = Math.atan2(-r.r0.y, r.r1.y);
            rz = 0.0;
        }
        return new Pose(t, new Vec3(rx, ry, rz));
    }

    @Override
    public String toString() {
        return String.format("Pose(pos=[%.3f, %.3f, %.3f], rot=[%.3f, %.3f, %.3f])",
                position.x, position.y, position.z,
                rotation.x, rotation.y, rotation.z);
    }
}

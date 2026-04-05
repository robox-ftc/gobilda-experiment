package org.firstinspires.ftc.robotarm.devices;

import org.firstinspires.ftc.robotarm.data.Mat3;
import org.firstinspires.ftc.robotarm.data.Mat4;
import org.firstinspires.ftc.robotarm.data.Pose;
import org.firstinspires.ftc.robotarm.data.Range;
import org.firstinspires.ftc.robotarm.data.Vec3;

/**
 * A joint that rotates around a fixed axis (hinge, turntable, servo horn).
 *
 * The local transform is a pure rotation about {@link #axis} by the
 * given angle, computed via Rodrigues' rotation formula.
 */
public class RevoluteJoint extends Joint {

    public RevoluteJoint(String name,
                         String parentLinkName, String childLinkName,
                         Vec3 axis, Pose parentToJoint, Pose jointToChild,
                         Range positionLimit, double maxVelocity, double maxAcceleration,
                         double damping, double homePosition) {
        super(name, JointType.REVOLUTE, parentLinkName, childLinkName, axis,
              parentToJoint, jointToChild,
              positionLimit, maxVelocity, maxAcceleration,
              damping, homePosition);
    }

    public RevoluteJoint(String name,
                         String parentLinkName, String childLinkName,
                         Vec3 axis, Pose parentToJoint, Pose jointToChild,
                         Range positionLimit, double maxVelocity, double maxAcceleration) {
        super(name, JointType.REVOLUTE, parentLinkName, childLinkName, axis,
              parentToJoint, jointToChild,
              positionLimit, maxVelocity, maxAcceleration);
    }

    public RevoluteJoint(String name,
                         String parentLinkName, String childLinkName,
                         Vec3 axis, Pose parentToJoint, Pose jointToChild,
                         Range positionLimit) {
        super(name, JointType.REVOLUTE, parentLinkName, childLinkName, axis,
                parentToJoint, jointToChild,
                positionLimit, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
    }



    /**
     * Rodrigues' rotation: R = I + sin(θ)·K + (1−cos(θ))·K²
     * where K is the skew-symmetric matrix of the unit axis.
     */
    @Override
    public Mat4 getLocalTransform(double position) {
        double c = Math.cos(position);
        double s = Math.sin(position);
        double v = 1.0 - c;
        double kx = axis.x, ky = axis.y, kz = axis.z;

        Mat3 rot = new Mat3(
            new Vec3(kx * kx * v + c,      kx * ky * v - kz * s,  kx * kz * v + ky * s),
            new Vec3(ky * kx * v + kz * s,  ky * ky * v + c,      ky * kz * v - kx * s),
            new Vec3(kz * kx * v - ky * s,  kz * ky * v + kx * s,  kz * kz * v + c)
        );
        return Mat4.fromRotationTranslation(rot, new Vec3());
    }

    @Override
    public int getDOF(){
        return 1;
    }

    @Override
    public String toString() {
        return String.format("RevoluteJoint(%s, axis=%s, limit=%s)", name, axis, positionRange);
    }
}

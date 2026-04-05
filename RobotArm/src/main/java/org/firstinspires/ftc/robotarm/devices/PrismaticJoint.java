package org.firstinspires.ftc.robotarm.devices;

import org.firstinspires.ftc.robotarm.data.Mat4;
import org.firstinspires.ftc.robotarm.data.Pose;
import org.firstinspires.ftc.robotarm.data.Range;
import org.firstinspires.ftc.robotarm.data.Vec3;

/**
 * A joint that slides along a fixed axis (linear slide, lead screw).
 *
 * The local transform is a pure translation along {@link #axis}
 * scaled by the given position.
 */
public class PrismaticJoint extends Joint {

    public PrismaticJoint(String name,
                          String parentLinkName, String childLinkName,
                          Vec3 axis, Pose parentToJoint, Pose jointToChild,
                          Range positionLimit, double maxVelocity, double maxAcceleration,
                          double damping, double homePosition) {
        super(name, JointType.PRISMATIC, parentLinkName, childLinkName, axis,
              parentToJoint, jointToChild,
              positionLimit, maxVelocity, maxAcceleration,
              damping, homePosition);
    }

    public PrismaticJoint(String name,
                          String parentLinkName, String childLinkName,
                          Vec3 axis, Pose parentToJoint, Pose jointToChild,
                          Range positionLimit, double maxVelocity, double maxAcceleration) {
        super(name, JointType.PRISMATIC, parentLinkName, childLinkName, axis,
              parentToJoint, jointToChild,
              positionLimit, maxVelocity, maxAcceleration);
    }

    @Override
    public Mat4 getLocalTransform(double position) {
        return Mat4.fromTranslation(axis.scale(position));
    }

    @Override
    public int getDOF(){
        return 1;
    }

    @Override
    public String toString() {
        return String.format("PrismaticJoint(%s, axis=%s, limit=%s)", name, axis, positionRange);
    }
}

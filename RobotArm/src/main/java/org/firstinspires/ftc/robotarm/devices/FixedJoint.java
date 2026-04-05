package org.firstinspires.ftc.robotarm.devices;

import org.firstinspires.ftc.robotarm.data.Mat4;
import org.firstinspires.ftc.robotarm.data.Pose;
import org.firstinspires.ftc.robotarm.data.Range;
import org.firstinspires.ftc.robotarm.data.Vec3;

/**
 * A rigid connection with no degrees of freedom.
 *
 * Used to attach links that don't move relative to each other
 * (e.g. a sensor mount, a fixed bracket, or a tool plate).
 * The local transform is always identity — position is ignored.
 */
public class FixedJoint extends Joint {

    public FixedJoint(String name,
                      String parentLinkName, String childLinkName,
                      Pose parentToJoint, Pose jointToChild) {
        super(name, JointType.FIXED, parentLinkName, childLinkName,
              new Vec3(),
              parentToJoint, jointToChild,
              new Range(0, 0), 0, 0,
              0, 0);
    }

    /** A fixed joint has zero degrees of freedom — it doesn't move. */
    @Override
    public int getDOF() {
        return 0;
    }

    @Override
    public Mat4 getLocalTransform(double position) {
        return Mat4.identity();
    }

    @Override
    public String toString() {
        return String.format("FixedJoint(%s)", name);
    }
}

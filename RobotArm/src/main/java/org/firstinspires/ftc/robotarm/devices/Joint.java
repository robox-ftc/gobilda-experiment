package org.firstinspires.ftc.robotarm.devices;

import org.firstinspires.ftc.robotarm.data.Mat4;
import org.firstinspires.ftc.robotarm.data.Pose;
import org.firstinspires.ftc.robotarm.data.Range;
import org.firstinspires.ftc.robotarm.data.Vec3;

/**
 * A single joint in a kinematic chain.
 * Note: This is a pure kinematic and dynamic description of the joint.
 * It does not store the state of the joint, or the target state.
 * It only provides the formula for the joint's movement.
 *
 * Follows the standard link–joint tree decomposition used in industrial
 * robotics (URDF / Drake / MoveIt): a joint connects a parent link to a
 * child link, has an axis of motion, and carries physical limits that come
 * from the real mechanism but are expressed in pure physics terms.
 *
 * Hardware-agnostic — does not mention servos, motors, encoders, or any
 * FTC SDK type. A separate {@link JointHardware} layer bridges this
 * model to actual actuators.
 * 
 * Stateless logic: Keeps the Joint class as a "Template." It should not store
 * the movement; it simply provides the formula for movement. This allows you
 * to reuse the same Joint object to calculate multiple potential positions.
 *
 * Links are referenced by name, not by object pointer. The {@link ArmModel}
 * owns the canonical Link registry; joints merely record which links they
 * connect. This keeps serialization simple and avoids circular references.
 *
 * <h3>Units</h3>
 * Angles in radians, lengths in meters.
 * REVOLUTE limits/velocity/acceleration are angular;
 * PRISMATIC are linear.
 */
public abstract class Joint {

    /** Unique name identifying this joint (e.g. "shoulder", "elbow"). */
    public final String name;

    /** Motion type (REVOLUTE, PRISMATIC, FIXED). */
    public final JointType type;

    // ── Kinematic tree (by name) ─────────────────────────────────────────
    /** Name of the parent link this joint is mounted on. */
    public final String parentLinkName;

    /** Name of the child link this joint drives. */
    public final String childLinkName;

    // ── Axis and mounting ───────────────────────────────────────────────

    /**
     * Joint axis in the parent link's frame.
     * Rotation axis for REVOLUTE, translation direction for PRISMATIC.
     * Unit vector, e.g. (0, 0, 1) for a Z-axis hinge.
     */
    public final Vec3 axis;
    
    /** Pose of the joint frame origin relative to the parent link frame. */
    public final Pose parentToJointPose;

    /** Pose of the child link frame relative to the joint frame at q = 0. */
    public final Pose jointToChildPose;

    // ── Physical limits ─────────────────────────────────────────────────

    /** Allowed range of the joint variable (radians or meters). Note this the 
     * limit of hard body limits, or desired safety ranges;
     * This is not what the servo can actually move and deliver
    */
    public final Range positionRange;

    /** Maximum absolute velocity (rad/s or m/s). */
    public final double maxVelocity;

    /** Maximum absolute acceleration (rad/s² or m/s²). */
    public final double maxAcceleration;

    /** Maximum absolute effort (N·m for REVOLUTE, N for PRISMATIC). This is the
     * safety desired limit of the joint, not the actual limit of the actuator.
     * What the mechanism can safely transmit.
    */
    public final double maxEffort;

    /** Viscous damping coefficient. */
    public final double damping;

    public Joint(String name, JointType type,
                 String parentLinkName, String childLinkName,
                 Vec3 axis, Pose parentToJointPose, Pose jointToChildPose,
                 Range positionRange, double maxVelocity, double maxAcceleration, double maxEffort,
                 double damping) {
        this.name = name;
        this.type = type;
        this.parentLinkName = parentLinkName;
        this.childLinkName = childLinkName;
        this.axis = axis;
        this.parentToJointPose = parentToJointPose;
        this.jointToChildPose = jointToChildPose;
        this.positionRange = positionRange;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.maxEffort = maxEffort;
        this.damping = damping;
    }

    public Joint(String name, JointType type,
                 String parentLinkName, String childLinkName,
                 Vec3 axis, Pose parentToJointPose, Pose jointToChildPose,
                 Range positionRange, double maxVelocity, double maxAcceleration) {
        this(name, type, parentLinkName, childLinkName, axis,
                parentToJointPose, jointToChildPose,
                positionRange, maxVelocity, maxAcceleration, Double.POSITIVE_INFINITY, 0.0);
    }
    public Joint(String name, JointType type,
                 String parentLinkName, String childLinkName,
                 Vec3 axis, Pose parentToJointPose, Pose jointToChildPose,
                 Range positionRange) {
        this(name, type, parentLinkName, childLinkName, axis,
                parentToJointPose, jointToChildPose,
                positionRange, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
    }


    /**
     * Calculates the 4x4 transformation matrix representing the joint's 
     * internal displacement or rotation based on its current state 'q'.
     * @param position The scalar joint variable (meters for PRISMATIC, radians for REVOLUTE).
     * @return A Mat4 representing the relative change between the joint's 
     * entrance and exit frames along the defined 'axis'.
     */
    public abstract Mat4 getLocalTransform(double position);


    public abstract int getDOF();

    @Override
    public String toString() {
        return String.format("Joint(%s, %s, parent=%s, child=%s, limit=%s)",
                name, type, parentLinkName, childLinkName, positionRange);
    }
}

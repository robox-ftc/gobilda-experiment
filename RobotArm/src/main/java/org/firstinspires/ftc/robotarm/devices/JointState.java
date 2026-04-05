package org.firstinspires.ftc.robotarm.devices;

/**
 * Instantaneous state of a single joint.
 *
 * Separates measured/runtime values from the static description in
 * {@link Joint}. Industrial stacks (MoveIt, Unitree ROS2) keep
 * state as a vector of (position, velocity, effort) per joint.
 */
public class JointState {

    /** Name of the joint this state belongs to. */
    public final String jointName;

    /** Current position — radians (REVOLUTE) or meters (PRISMATIC). */
    public final double position;

    /** Current velocity — rad/s or m/s. */
    public final double velocity;

    /**
     * Estimated effort (torque in N·m for REVOLUTE, force in N for
     * PRISMATIC). 0 when not measured or not estimated.
     */
    public final double effort;

    public JointState(String jointName, double position, double velocity, double effort) {
        this.jointName = jointName;
        this.position = position;
        this.velocity = velocity;
        this.effort = effort;
    }

    public JointState(String jointName, double position, double velocity) {
        this(jointName, position, velocity, 0.0);
    }

    @Override
    public String toString() {
        return String.format("JointState(%s, pos=%.3f, vel=%.3f)", jointName, position, velocity);
    }
}

package org.firstinspires.ftc.robotarm.devices;

/**
 * A desired target for a single joint.
 *
 * Simple value object pairing a joint name with a target position.
 * Intentionally a plain class (not interface) to match the project's
 * data-holder style.
 */
public class JointCommand {
    public final String jointName;
    public final double targetPosition;

    public JointCommand(String jointName, double targetPosition) {
        this.jointName = jointName;
        this.targetPosition = targetPosition;
    }

    @Override
    public String toString() {
        return String.format("JointCommand(%s -> %.3f)", jointName, targetPosition);
    }
}

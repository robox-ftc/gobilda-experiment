package org.firstinspires.ftc.robotarm.devices;

import org.firstinspires.ftc.robotarm.data.Pose;

import java.util.Map;

/**
 * Snapshot of the full arm's runtime state.
 *
 * Contains the current state of every joint plus the forward-kinematics
 * result for the end-effector. This is the "robot state" layer — it tells
 * you what the arm is doing right now.
 */
public class ArmState {

    /** Per-joint state, keyed by joint name. */
    public final Map<String, JointState> jointStates;

    /**
     * Computed pose of the tool / end-effector link in the base frame.
     * Forward-kinematics result derived from current joint positions
     * and the arm model.
     */
    public final Pose endEffectorPose;

    public ArmState(Map<String, JointState> jointStates, Pose endEffectorPose) {
        this.jointStates = jointStates;
        this.endEffectorPose = endEffectorPose;
    }

    @Override
    public String toString() {
        return String.format("ArmState(jointStates=%d, endEffectorPose=%s)", jointStates.size(), endEffectorPose);
    }
}

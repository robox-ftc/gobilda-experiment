package org.firstinspires.ftc.robotarm.devices;

import org.firstinspires.ftc.robotarm.data.Mat4;
import org.firstinspires.ftc.robotarm.data.Pose;
import org.firstinspires.ftc.robotarm.data.Vec3;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Complete kinematic and physical description of a robot arm.
 *
 * A tree of {@link Link}s connected by {@link Joint}s, rooted at a
 * base link and terminating at a tool (end-effector) link. This is
 * the "robot description" layer — it tells you what the arm is,
 * not what it is currently doing.
 *
 * <p>Analogous to a URDF file in ROS or a MultibodyPlant in Drake.
 */
public class ArmModel {

    /** Human-readable name for this arm configuration. */
    public final String name;

    /** All links in the kinematic tree, in parent-first order. */
    public final List<Link> links;

    /** All joints in the kinematic tree, in parent-first order. */
    public final List<Joint> joints;

    /** Name of the root (fixed-to-world) link. */
    public final String rootLinkName;

    /** Name of the end-effector / tool link. */
    public final String toolLinkName;

    public ArmModel(String name,
                    List<Link> links, List<Joint> joints,
                    String rootLinkName, String toolLinkName) {
        this.name = name;
        this.links = links;
        this.joints = joints;
        this.rootLinkName = rootLinkName;
        this.toolLinkName = toolLinkName;
    }

    // ── Lookups ──────────────────────────────────────────────────────────

    /** Find a link by name, or null if not found. */
    public Link getLinkByName(String name) {
        for (Link link : links) {
            if (link.name.equals(name)) return link;
        }
        return null;
    }

    /** Find a joint by name, or null if not found. */
    public Joint getJointByName(String name) {
        for (Joint joint : joints) {
            if (joint.name.equals(name)) return joint;
        }
        return null;
    }

    // ── Forward Kinematics ───────────────────────────────────────────────

    /**
     * Compute the end-effector (tool) pose in the base frame.
     *
     * <p>Walks the joint list in parent-first order, chaining:
     * <pre>
     *   T = T · parentToJoint · localTransform(q) · jointToChild
     * </pre>
     * for each joint. The result is the cumulative transform from the
     * root link frame to the tool link frame.
     *
     * @param jointPositions current position of each movable joint,
     *                       keyed by {@link Joint#name}. Fixed joints
     *                       are skipped automatically.
     * @return the tool pose in the base (root link) frame
     */
    public Pose forwardKinematics(Map<String, Double> jointPositions) {
        return Pose.fromMat4(forwardKinematicsMat4(jointPositions));
    }

    /**
     * Same as {@link #forwardKinematics} but returns the raw 4×4
     * homogeneous transform (avoids Euler extraction when not needed).
     */
    public Mat4 forwardKinematicsMat4(Map<String, Double> jointPositions) {
        Mat4 T = Mat4.identity();
        for (Joint joint : joints) {
            T = T.mul(joint.parentToJointPose.toMat4());

            double q = 0.0;
            if (joint.getDOF() > 0) {
                Double pos = jointPositions.get(joint.name);
                if (pos != null) q = pos;
            }
            T = T.mul(joint.getLocalTransform(q));

            T = T.mul(joint.jointToChildPose.toMat4());
        }
        return T;
    }

    /**
     * Compute the world-frame transform for every child-link frame in
     * the chain. Useful for gravity compensation, visualization, and
     * collision checking.
     *
     * @param jointPositions current joint positions (same as FK)
     * @return ordered map from child link name → transform in base frame
     */
    public Map<String, Mat4> getLinkTransforms(Map<String, Double> jointPositions) {
        Map<String, Mat4> transforms = new LinkedHashMap<>();
        transforms.put(rootLinkName, Mat4.identity());

        Mat4 T = Mat4.identity();
        for (Joint joint : joints) {
            T = T.mul(joint.parentToJointPose.toMat4());

            double q = 0.0;
            if (joint.getDOF() > 0) {
                Double pos = jointPositions.get(joint.name);
                if (pos != null) q = pos;
            }
            T = T.mul(joint.getLocalTransform(q));
            T = T.mul(joint.jointToChildPose.toMat4());

            transforms.put(joint.childLinkName, T);
        }
        return transforms;
    }

    /**
     * Compute the position of every link's center of mass in the base
     * frame. Needed for gravity torque estimation.
     *
     * @param jointPositions current joint positions
     * @return map from link name → CoM position in base frame
     */
    public Map<String, Vec3> getLinkCentersOfMass(Map<String, Double> jointPositions) {
        Map<String, Mat4> linkTransforms = getLinkTransforms(jointPositions);
        Map<String, Vec3> comPositions = new LinkedHashMap<>();
        for (Link link : links) {
            Mat4 T = linkTransforms.get(link.name);
            if (T != null) {
                comPositions.put(link.name, T.transformPoint(link.centerOfMass));
            }
        }
        return comPositions;
    }

    // ── Aggregate properties ─────────────────────────────────────────────

    /**
     * Total degrees of freedom across all joints.
     * Sums each joint's {@link Joint#getDOF()}, so FIXED joints
     * contribute 0, REVOLUTE and PRISMATIC contribute 1 each, and
     * any future multi-DOF joint type will contribute correctly.
     */
    public int getTotalDOF() {
        int dof = 0;
        for (Joint j : joints) {
            dof += j.getDOF();
        }
        return dof;
    }

    @Override
    public String toString() {
        return String.format("ArmModel(%s, %d links, %d joints, %d DOF)",
                name, links.size(), joints.size(), getTotalDOF());
    }
}

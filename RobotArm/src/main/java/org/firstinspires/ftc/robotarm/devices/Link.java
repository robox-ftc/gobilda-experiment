package org.firstinspires.ftc.robotarm.devices;

import org.firstinspires.ftc.robotarm.data.Mat3;

import org.firstinspires.ftc.robotarm.data.Vec3;

/**
 * A rigid link (segment) in a kinematic chain.
 *
 * Links are data containers — they hold static geometry and mass
 * properties relative to their own coordinate frame. This matches
 * how URDF (ROS) and MuJoCo model links.
 */
public class Link {

    /** Unique name identifying this link (e.g. "base", "forearm", "hand"). */
    public final String name;

    /** Length along the principal axis, in meters. */
    public final double lengthMeters;

    /** Total mass, in kilograms. */
    public final double massKg;

    /**
     * Center of mass in the link's own coordinate frame.
     * For a uniform bar this is typically at half the length along the
     * principal axis.
     */
    public final Vec3 centerOfMass;

    /**
     * 3 × 3 rotational inertia tensor about the center of mass, in the
     * link's own frame. Null when not yet measured — required later for
     * torque estimation and gravity compensation.
     */
    public final Mat3 inertia;

    public Link(String name, double lengthMeters, double massKg,
                Vec3 centerOfMass, Mat3 inertia) {
        this.name = name;
        this.lengthMeters = lengthMeters;
        this.massKg = massKg;
        this.centerOfMass = centerOfMass;
        this.inertia = inertia;
    }

    public Link(String name, double lengthMeters, double massKg,
                Vec3 centerOfMass) {
        this(name, lengthMeters, massKg, centerOfMass, null);
    }

    /**
     * Usually for a bar with a Revolute joint
     * The origin (0,0,0) is from the anchor bolt.
     * X-axis along is aligned the length of the bar, extend from the anchor bolt.
     * In this settings, the center of mass is at the middle of the bar, x/2;
     * Z-axis is always the axis of rotation or translation.
     * For a revolute joint, the Z-axis is aligned the center of the motor shaft.
     * Y-axis is determined by the Right-Hand Rule to ensure a Right Hand Coordinate System.
     *
     * For a wasit or rotating wrist, with a twist joint, we have a Axial joint/Torsional Joint,
     * i.e. The shaft points forward along the same direction of the arm itself
     * Origin, still at the center of motor shaft
     * Z-axis is still the aixs of rotation, pointing forward, collinear with the length of the shaft
     * X-axis is perpendicular to Z-axis, up or sideways. Y-axis follow RHR
     *
     * @param name
     * @param lengthMeters
     * @param massKg
     */
    public Link(String name, double lengthMeters, double massKg) {
        this(name, lengthMeters, massKg,
             new Vec3(lengthMeters / 2.0, 0, 0));
    }

    @Override
    public String toString() {
        return String.format("Link(%s, len=%.3fm, mass=%.3fkg)", name, lengthMeters, massKg);
    }
}

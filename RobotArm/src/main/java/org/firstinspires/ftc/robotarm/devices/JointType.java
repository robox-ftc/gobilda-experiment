package org.firstinspires.ftc.robotarm.devices;

/**
 * Motion type of a joint, following standard robotics convention.
 *
 * REVOLUTE  – rotates around an axis (servo, hinge, turntable).
 * PRISMATIC – translates along an axis (linear slide, lead screw).
 * FIXED     – rigid attachment, no relative motion.
 */
public enum JointType {
    REVOLUTE,
    PRISMATIC,
    FIXED
}

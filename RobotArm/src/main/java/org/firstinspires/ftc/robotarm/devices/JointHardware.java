package org.firstinspires.ftc.robotarm.devices;

/**
 * Hardware adapter for a single joint's actuator and sensor.
 *
 * This is the only layer that touches the FTC SDK (servos, motors,
 * encoders, Control Hub). Everything above works in pure physics
 * units and knows nothing about PWM channels, encoder CPR, gear
 * ratios, or vendor-specific APIs.
 *
 * <p>Subclass for each actuator type:
 * <ul>
 *   <li>A goBILDA servo on a real robot</li>
 *   <li>A geared DC motor with encoder</li>
 *   <li>A simulated joint in a desktop test harness</li>
 * </ul>
 */
public abstract class JointHardware {

    /** Name of the device in the FTC {@code HardwareMap} (e.g. "servo0"). */
    public final String deviceName;

    public JointHardware(String deviceName) {
        this.deviceName = deviceName;
    }

    /** Read the current position — radians or meters, already converted from raw units. */
    public abstract double readPosition();

    /** Read the current velocity — rad/s or m/s. */
    public abstract double readVelocity();

    /** Command the actuator to move toward the given position. */
    public abstract void writePositionTarget(double target);

    /** Cut power / hold position (depending on actuator type). */
    public abstract void stop();
}

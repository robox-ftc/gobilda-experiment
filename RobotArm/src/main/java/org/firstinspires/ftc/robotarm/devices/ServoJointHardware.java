package org.firstinspires.ftc.robotarm.devices;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * {@link JointHardware} backed by a standard FTC position-controlled
 * {@link Servo} (e.g. goBILDA Torque Servo 2000-0025-0002).
 *
 * <p>Handles the conversion between the model's physics units (radians)
 * and the servo's dimensionless 0.0–1.0 PWM range. The mapping is
 * linear:
 * <pre>
 *   servo 0.0  ↔  minAngleRad
 *   servo 1.0  ↔  maxAngleRad
 * </pre>
 *
 * <p>Standard servos have no velocity feedback, so
 * {@link #readVelocity()} always returns 0.
 *
 * <p>Usage from a {@link JointHardwareFactory}:
 * <pre>{@code
 * (joint, hwMap) -> new ServoJointHardware(
 *     "shoulder_servo",                   // HardwareMap device name
 *     hwMap.get(Servo.class, "shoulder_servo"),
 *     Math.toRadians(0),                  // servo 0.0 = 0°
 *     Math.toRadians(300))                // servo 1.0 = 300°
 * }</pre>
 */
public class ServoJointHardware extends JointHardware {

    private final Servo servo;

    /** Angle (radians) corresponding to servo position 0.0. */
    private final double minAngleRad;

    /** Angle (radians) corresponding to servo position 1.0. */
    private final double maxAngleRad;

    private final double rangeRad;

    /**
     * @param deviceName  the device name in the FTC {@code HardwareMap}
     * @param servo       the FTC Servo obtained from {@code hardwareMap.get(Servo.class, ...)}
     * @param minAngleRad physical angle when the servo is at position 0.0
     * @param maxAngleRad physical angle when the servo is at position 1.0
     */
    public ServoJointHardware(String deviceName, Servo servo,
                              double minAngleRad, double maxAngleRad) {
        super(deviceName);
        this.servo = servo;
        this.minAngleRad = minAngleRad;
        this.maxAngleRad = maxAngleRad;
        this.rangeRad = maxAngleRad - minAngleRad;
    }

    /**
     * Convenience constructor using the goBILDA Torque Servo's
     * default 300° range (0 to 5π/3 radians).
     */
    public ServoJointHardware(String deviceName, Servo servo) {
        this(deviceName, servo, 0.0, Math.toRadians(300));
    }

    @Override
    public double readPosition() {
        return minAngleRad + servo.getPosition() * rangeRad;
    }

    @Override
    public double readVelocity() {
        return 0.0;
    }

    @Override
    public void writePositionTarget(double targetRad) {
        double servoPos = (targetRad - minAngleRad) / rangeRad;
        servo.setPosition(clamp(servoPos, 0.0, 1.0));
    }

    @Override
    public void stop() {
        // Standard servos hold their last commanded position;
        // PWM disable would let it go limp, but that's usually
        // unsafe for an arm joint under gravity.
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    @Override
    public String toString() {
        return String.format("ServoJointHardware(%s, range=[%.1f°, %.1f°])",
                deviceName, Math.toDegrees(minAngleRad), Math.toDegrees(maxAngleRad));
    }
}

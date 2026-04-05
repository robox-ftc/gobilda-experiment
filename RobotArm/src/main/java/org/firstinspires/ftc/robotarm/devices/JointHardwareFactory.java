package org.firstinspires.ftc.robotarm.devices;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Creates the correct {@link JointHardware} for a given {@link Joint}
 * using devices looked up from the FTC {@link HardwareMap}.
 *
 * <p>Each robot configuration supplies its own factory, mapping joint
 * types and names to the appropriate actuator/sensor wrappers.
 *
 * <pre>{@code
 * // Map joint names to HardwareMap device names
 * Map<String, String> devices = Map.of(
 *     "shoulder", "servo0",
 *     "elbow",    "servo1");
 *
 * JointHardwareFactory factory = (joint, hwMap) -> {
 *     String dev = devices.get(joint.name);
 *     return new ServoJointHardware(dev,
 *         hwMap.get(Servo.class, dev));
 * };
 * }</pre>
 */
@FunctionalInterface
public interface JointHardwareFactory {

    /**
     * Create the hardware adapter for the given joint.
     *
     * @param joint       the kinematic joint from the {@link ArmModel}
     * @param hardwareMap the FTC-managed hardware registry
     * @return a {@link JointHardware} wired to the real (or simulated) actuator
     */
    JointHardware create(Joint joint, HardwareMap hardwareMap);
}

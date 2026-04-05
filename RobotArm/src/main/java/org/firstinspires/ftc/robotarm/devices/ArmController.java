package org.firstinspires.ftc.robotarm.devices;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Drives an arm to desired joint targets.
 *
 * Translates {@link JointCommand}s into hardware calls via
 * {@link JointHardware}, enforces limits from the {@link ArmModel},
 * and can be extended with motion profiling or safety checks.
 */
public class ArmController {

    public final ArmModel model;
    public final Map<String, JointHardware> hardware;

    /**
     * Creates an ArmController from an {@link ArmModel} and the FTC
     * {@link HardwareMap}. Iterates over every movable joint in the
     * model and delegates to the {@link JointHardwareFactory} to
     * create the correct {@link JointHardware} for each one.
     *
     * <p>This is the primary constructor for real OpModes:
     * <pre>{@code
     * ArmController arm = new ArmController(model, hardwareMap,
     *     (joint, hwMap) -> new ServoJointHardware("servo0",
     *         hwMap.get(Servo.class, "servo0")));
     * }</pre>
     */
    public ArmController(ArmModel model, HardwareMap hardwareMap,
                         JointHardwareFactory factory) {
        this.model = model;
        this.hardware = new LinkedHashMap<>();
        for (Joint joint : model.joints) {
            if (joint.getDOF() > 0) {
                hardware.put(joint.name, factory.create(joint, hardwareMap));
            }
        }
    }

    /**
     * Direct constructor for testing or simulation, where no FTC
     * {@link HardwareMap} is available.
     */
    public ArmController(ArmModel model, Map<String, JointHardware> hardware) {
        this.model = model;
        this.hardware = hardware;
    }

    /** Send position targets for one or more joints. */
    public void setJointTargets(List<JointCommand> commands) {
        for (JointCommand cmd : commands) {
            JointHardware hw = hardware.get(cmd.jointName);
            if (hw != null) {
                hw.writePositionTarget(cmd.targetPosition);
            }
        }
    }

    /** Convenience: target a single joint. */
    public void setJointTarget(String jointName, double targetPosition) {
        setJointTargets(List.of(new JointCommand(jointName, targetPosition)));
    }

    /** Immediately stop all joints. */
    public void stop() {
        for (JointHardware hw : hardware.values()) {
            hw.stop();
        }
    }

    /** Read the current state of the arm, including FK-computed tool pose. */
    public ArmState getState() {
        LinkedHashMap<String, JointState> states = new LinkedHashMap<>();
        Map<String, Double> positions = new LinkedHashMap<>();

        for (Map.Entry<String, JointHardware> entry : hardware.entrySet()) {
            String name = entry.getKey();
            JointHardware hw = entry.getValue();
            double pos = hw.readPosition();
            double vel = hw.readVelocity();
            states.put(name, new JointState(name, pos, vel));
            positions.put(name, pos);
        }

        org.firstinspires.ftc.robotarm.data.Pose toolPose = model.forwardKinematics(positions);
        return new ArmState(states, toolPose);
    }

    @Override
    public String toString() {
        return String.format("ArmController(%s, %d hw)", model.name, hardware.size());
    }
}

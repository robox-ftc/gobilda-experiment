
package org.firstinspires.ftc.testbed;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotarm.data.*;
import org.firstinspires.ftc.robotarm.devices.*;

import java.util.ArrayList;
import java.util.List;

/**
 * Test the robot arm.
 *
 * 1. Initialize the arm model and hardware
 * 2. Read the arm commands from the gamepad
 * 3. Set the joint targets for the arm
 * 4. Update the arm state
 * 5. Display the arm state
 */
@TeleOp(name = "toy arm", group = "Robot")
public class ToyRobotArmTest extends OpMode {
    private ArmController armController = null;
    private ArmModel armModel = null;
    private ArmState armState = null;

    /**
     * Map motors, servos, sensors etc.; After press Init button. Must be short.
     */
    @Override
    public void init() {
        // parent first links and joints
        List<Link> links = new ArrayList<>();
        List<Joint> joints = new ArrayList<>();

        Link base = new Link("base", 0.0, Double.POSITIVE_INFINITY);
        Link waist = new Link("waist", 0.05, 0.02, new Vec3(0, 0, 0.01));
        Link shoulder = new Link("shoulder", 0.076, 0.75);
        Link elbow = new Link("elbow", 0.076, 0.05, new Vec3(0.076/3*2, 0, 0));
        Link wrist = new Link("wrist", 0.09, 0.75, new Vec3(0.01, 0, 0.09/2));

        links.add(base);
        links.add(waist);
        links.add(shoulder);
        links.add(elbow);
        links.add(wrist);

       /* joints.add(new RevoluteJoint("waist", "base", "waist", new Vec3(0, 0, 1), new Vec3(0,0,0), new Range(-Math.PI/2, Math.PI/2)));
        joints.add(new RevoluteJoint("shoulder", "waist", "shoulder", new Vec3(0, 0, 1), new Vec3(0,0,0), new Range(-Math.PI/2, Math.PI/2)));
        joints.add(new RevoluteJoint("elbow", "shoulder", "elbow", new Vec3(0, 0, 1), new Vec3(0,0,0), new Range(-Math.PI/2, Math.PI/2)));
        joints.add(new RevoluteJoint("wrist", "elbow", "wrist", new Vec3(0, 0, 1), new Vec3(0,0,0), new Range(-Math.PI/2, Math.PI/2)));
*/
        armModel = new ArmModel("testArm", links, joints, "base", "tool");

        armController = new ArmController(armModel, hardwareMap, (joint, hwMap) -> {
            String device = joint.name + "Servo";  // convention: joint name == device name + "Servo"
            return new ServoJointHardware(device,
                    hwMap.get(Servo.class, device));
        });

        armState = armController.getState();
    }

    /**
     * Collect camera vision data; calibrate sensors etc. Before press "Play". after init()
     */
    @Override
    public void init_loop() {
        armState = armController.getState();
        telemetry.addData("Arm State", armState.toString());
    }

    /**
     * After press "PLay", starts counters, timers etc.
     */
    @Override
    public void start() {
    }


    /**
     * The loop of driving, read controls, sensors and take actions.
     */
    @Override
    public void loop() {
        List<JointCommand> armCommands = readArmCommands(gamepad1, gamepad2);
        armController.setJointTargets(armCommands);
    }

    /**
     * Power down, close camera, save collected data etc.
     */
    @Override
    public void stop() {
        armController.stop();
    }

    private List<JointCommand> readArmCommands(Gamepad gamepad1, Gamepad gamepad2) {
        List<JointCommand> commands = new ArrayList<>();
        /* 
        commands.add(new JointCommand("waist", gamepad1.left_stick_y));
        commands.add(new JointCommand("shoulder", gamepad1.right_stick_y));
        commands.add(new JointCommand("elbow", gamepad1.right_stick_x));
        commands.add(new JointCommand("wrist", gamepad2.right_trigger));
        */
        return commands;
    }
}
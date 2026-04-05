
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
 * In this test class will we do not use any model, just hardcode the servo motors and control them directly.
 */
@TeleOp(name = "Easy Toy Arm", group = "Robot")
public class ToyRobotArmTestEasy extends OpMode {
    private Servo waistServo = null;
    private Servo shoulderServo = null;
    private Servo elbowServo = null;
    private Servo wristServo = null;
    private Servo toolServo = null;

    /**
     * Map motors, servos, sensors etc.; After press Init button. Must be short.
     */
    @Override
    public void init() {
        waistServo = hardwareMap.get(Servo.class, "waistServo");
        shoulderServo = hardwareMap.get(Servo.class, "shoulderServo");
        elbowServo = hardwareMap.get(Servo.class, "elbowServo");
        wristServo = hardwareMap.get(Servo.class, "wristServo");
        toolServo = hardwareMap.get(Servo.class, "toolServo");
    }

    /**
     * Collect camera vision data; calibrate sensors etc. Before press "Play". after init()
     */
    @Override
    public void init_loop() {
        waistServo.setPosition(0.5);
        shoulderServo.setPosition(0.5);
        elbowServo.setPosition(0.5);
        wristServo.setPosition(0.5);
        toolServo.setPosition(0.5);
    }

    public void resetPosition(){
        waistServo.setPosition(0.5);
        shoulderServo.setPosition(0.5);
        elbowServo.setPosition(0.5);
        wristServo.setPosition(0.5);
        toolServo.setPosition(0.5);
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
        List<Command> armCommands = readArmCommands(gamepad1, gamepad2);
        executeArmCommands(armCommands);
    }

    /**
     * Power down, close camera, save collected data etc.
     */
    @Override
    public void stop() {
        resetPosition();
    }


    private class Command {
        public Servo servo = null;
        public double targetPosition = 0.5;
        public Command(Servo servo, double targetPosition) {
            this.servo = servo;
            this.targetPosition = targetPosition;
        }
    }

    /*
    Control mappings:
    Left stick X: waist rotation
    Left stick Y: shoulder rotation
    Right stick Y: elbow rotation
    Right stick X: wrist rotation
    Right trigger: tool on/off
     */
    private List<Command> readArmCommands(Gamepad gamepad1, Gamepad gamepad2) {
        List<Command> commands = new ArrayList<>();
        commands.add(new Command(this.waistServo, neg11To01(gamepad1.left_stick_x)));
        commands.add(new Command(this.shoulderServo, neg11To01(gamepad1.left_stick_y)));
        commands.add(new Command(this.elbowServo, neg11To01(gamepad1.right_stick_y)));
        commands.add(new Command(this.wristServo, neg11To01(gamepad1.right_stick_x)));
        commands.add(new Command(this.toolServo, neg11To01(gamepad2.right_trigger)));
        return commands;
    }

    //convert -1 to 1 to 0 to 1 range
    private double neg11To01(double value){
        return (value + 1.0)/2.0; 
    }

    private void executeArmCommands(List<Command> commands) {
        for (Command command : commands) {
            command.servo.setPosition(command.targetPosition);
        }
    }
}
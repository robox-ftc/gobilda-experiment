package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Motor-Servo Test", group = "Robot")
public class Test extends LinearOpMode {

    public static final double SAFE_CONSTANT = 0.5; // arbitrary constant

    public DcMotor motor = null; // motor
    public CRServo servo = null; // servo


    public void runOpMode() {
        motor = hardwareMap.get(DcMotor.class, "motor");
        servo = hardwareMap.get(CRServo.class, "servo");
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        while (opModeIsActive()) {
            motor.setPower(gamepad1.left_stick_y * SAFE_CONSTANT);
            servo.setPower(gamepad1.right_stick_y * SAFE_CONSTANT);

            telemetry.addData("Motor: ", gamepad1.left_stick_y);
            telemetry.addData("Servo: ", gamepad1.right_stick_y);
        }
    }
}

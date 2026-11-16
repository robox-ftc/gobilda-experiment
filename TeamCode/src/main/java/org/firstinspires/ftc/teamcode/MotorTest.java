package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Motor Test", group = "Robot")
public class MotorTest extends LinearOpMode {

    public static final double SAFE_CONSTANT = 0.5; // arbitrary constant

    public DcMotor motor = null; // motor

    public void runOpMode() {
        motor = hardwareMap.get(DcMotor.class, "motor");
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        while (opModeIsActive()) {
            motor.setPower(gamepad1.left_stick_y * SAFE_CONSTANT);

            telemetry.addData("Motor: ", gamepad1.left_stick_y);
        }
    }
}

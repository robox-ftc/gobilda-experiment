package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Launch Test", group = "Robot")
public class LaunchTest extends LinearOpMode {
    public DcMotor launcher = null; // launcher

    @Override
    public void runOpMode() {
        launcher = hardwareMap.get(DcMotor.class, "launch");
        launcher.setDirection(DcMotor.Direction.FORWARD);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        while (opModeIsActive()) {
            launcher.setPower(10);
            telemetry.addData("Launch Speed: ", gamepad1.left_stick_y);
        }
    }
}

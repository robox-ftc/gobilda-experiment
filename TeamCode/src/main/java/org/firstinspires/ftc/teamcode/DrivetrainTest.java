
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Utils.Drivetrain;
import org.firstinspires.ftc.teamcode.Utils.DrivetrainControls;

@TeleOp(name = "DriveTest", group = "Robot")
//@Disabled
public class DrivetrainTest extends OpMode {
    private Drivetrain drivetrain = null;


    public void init() {
        drivetrain = new Drivetrain(hardwareMap, telemetry, false);
    }

    public void loop() {
        DrivetrainControls drivetrainControls = readDrivetrainControls(gamepad1, gamepad2);

        double[] drivetrainPowers = computeDriveTrainPower(drivetrainControls);
        this.drivetrain.setPowers(drivetrainPowers);

        drivetrain.run(false);
    }

    private DrivetrainControls readDrivetrainControls(Gamepad gamepad1, Gamepad gamepad2) {
        double x = gamepad1.left_stick_x + gamepad2.left_stick_x;
        double y = -(gamepad1.left_stick_y + gamepad2.left_stick_y);
        double a = gamepad1.right_stick_x + gamepad2.right_stick_x;
        return new DrivetrainControls(x, y, a);
    }

    private double[] computeDriveTrainPower(DrivetrainControls controls) {

        double frontLeftPower  = controls.translationYPower + controls.translationXPower + controls.rotationPower;
        double frontRightPower = controls.translationYPower - controls.translationXPower - controls.rotationPower;
        double rearLeftPower   = controls.translationYPower - controls.translationXPower + controls.rotationPower;
        double rearRightPower  = controls.translationYPower + controls.translationXPower - controls.rotationPower;

        double maxPower = Math.max(1.0, Math.max(
                Math.abs(frontLeftPower),
                Math.max(Math.abs(frontRightPower),
                        Math.max(Math.abs(rearLeftPower),
                                Math.abs(rearRightPower)))
        ));

        frontLeftPower  /= maxPower;
        frontRightPower /= maxPower;
        rearRightPower  /= maxPower;
        rearLeftPower   /= maxPower;

        return new double[]{frontLeftPower, frontRightPower, rearRightPower, rearLeftPower};
    }
}
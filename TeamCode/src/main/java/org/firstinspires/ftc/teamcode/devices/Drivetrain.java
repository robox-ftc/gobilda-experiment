package org.firstinspires.ftc.teamcode.devices;

import static org.firstinspires.ftc.teamcode.utils.Utils.applyAction;
import static org.firstinspires.ftc.teamcode.utils.Utils.applyActions;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.data.Vec2;

import java.util.Arrays;

public class Drivetrain {

    // Declare OpMode members.
    private DcMotorEx frontLeftDrive = null;
    private DcMotorEx frontRightDrive = null;
    private DcMotorEx rearLeftDrive = null;
    private DcMotorEx rearRightDrive = null;
    private DcMotorEx[] driveMotors = new DcMotorEx[4];
    private Telemetry telemetry = null;

    public Drivetrain(HardwareMap hardwareMap, Telemetry telemetry, boolean auto){
        this.telemetry = telemetry;
        init(hardwareMap, auto);
    }

    public void init(HardwareMap hardwareMap, boolean auto){
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "lfdrive");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "rfdrive");
        rearLeftDrive = hardwareMap.get(DcMotorEx.class, "lbdrive");
        rearRightDrive = hardwareMap.get(DcMotorEx.class, "rbdrive");
        driveMotors[0] = frontLeftDrive;
        driveMotors[1] = frontRightDrive;
        driveMotors[2] = rearRightDrive;
        driveMotors[3] = rearLeftDrive;
        applyAction(driveMotors, (motor) -> motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER));
        applyAction(driveMotors, (motor) -> motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE));
        applyAction(driveMotors, (motor) -> motor.setPower(0.0));
    }

    public void setPowers(double [] powers) {
        if (powers != null && powers.length == 4)
            applyActions(driveMotors, (motor, i) -> motor.setPower(powers[i]));
    }

    public void run(DrivetrainControls controls){
        setPowers(computeDriveTrainPower(controls));
    }
    
    public void rotate(double power){
        DrivetrainControls dr = new DrivetrainControls(){{
                translationXPower = 0;
                translationYPower = 0;
                rotationPower = power;
        }};

        double[] powers = computeDriveTrainPower(dr);
        setPowers(powers);
    }

    public void translate(Vec2 direction, double power){
        Vec2 d = direction.normalize();
        DrivetrainControls dr = new DrivetrainControls(d.x * power,
                d.y * power, 0.0);

        double[] powers = computeDriveTrainPower(dr);
        setPowers(powers);
    }

    private double[] computeDriveTrainPower(DrivetrainControls controls) {
        double frontLeftPower  = controls.translationYPower + controls.translationXPower + controls.rotationPower;
        double frontRightPower = -controls.translationYPower + controls.translationXPower + controls.rotationPower;
        double rearLeftPower   = controls.translationYPower - controls.translationXPower + controls.rotationPower;
        double rearRightPower  = -controls.translationYPower - controls.translationXPower + controls.rotationPower;

        double maxPower = Math.max(Math.abs(frontLeftPower),
                Math.max(Math.abs(frontRightPower), Math.max(Math.abs(rearLeftPower), Math.abs(rearRightPower))));

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            rearRightPower /= maxPower;
            rearLeftPower /= maxPower;
        }

        return new double[]{frontLeftPower, frontRightPower, rearRightPower, rearLeftPower};
    }
}

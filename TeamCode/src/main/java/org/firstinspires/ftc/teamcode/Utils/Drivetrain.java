package org.firstinspires.ftc.teamcode.Utils;

import static org.firstinspires.ftc.teamcode.Utils.Utils.applyAction;
import static org.firstinspires.ftc.teamcode.Utils.Utils.applyActions;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;

public class Drivetrain implements IDevice {

    // Declare OpMode members.
    private DcMotorEx frontLeftDrive = null;
    private DcMotorEx frontRightDrive = null;
    private DcMotorEx rearLeftDrive = null;
    private DcMotorEx rearRightDrive = null;
    private DcMotorEx[] driveMotors = new DcMotorEx[4];
    private double[] targetPowers = new double[4];
    private int[] targetPositions = new int[4];
    private DrivetrainControls controls = new DrivetrainControls();

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
        // for now, to be changed
        applyAction(driveMotors, (motor) -> motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE));
        applyAction(driveMotors, (motor) -> motor.setPower(0.0));
    }

    public void setPowers(double [] powers) {
        if (powers != null && powers.length == 4)
            targetPowers = powers;
    }


    public void run(boolean auto){
        // For now we only implemented non-auto mode.
        if (!auto) {
            setPowers(computeDriveTrainPower(this.controls));
        }
        this.telemetry.addData("dr pwr", Arrays.toString(this.targetPowers));
        applyActions(driveMotors, (motor, i) -> motor.setPower(this.targetPowers[i]));
    }

    public void readControls(GamePadReadings oldReadings, GamePadReadings newReadings)
    {
        this.controls.translationXPower = newReadings.leftStickX;
        this.controls.translationYPower = -newReadings.leftStickY;
        this.controls.rotationPower = newReadings.rightStickX;
        this.telemetry.addData("sticks ",  this.controls.toString());
    }

    private double[] computeDriveTrainPower(DrivetrainControls controls) {
        double frontLeftPower  = controls.translationYPower + controls.translationXPower + controls.rotationPower;
        double frontRightPower = -controls.translationYPower + controls.translationXPower + controls.rotationPower;
        double rearLeftPower   = controls.translationYPower - controls.translationXPower + controls.rotationPower;
        double rearRightPower  = -controls.translationYPower - controls.translationXPower + controls.rotationPower;

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

package org.firstinspires.ftc.teamcode.Utils;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static org.firstinspires.ftc.teamcode.Utils.Utils.applyAction;
import static org.firstinspires.ftc.teamcode.Utils.Utils.applyActions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Drivetrain implements IDevice {

    // Declare OpMode members.
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor rearLeftDrive = null;
    private DcMotor rearRightDrive = null;
    private DcMotor[] driveMotors = new DcMotor[4];
    private double[] targetPowers = new double[4];
    private DrivetrainControls controls = new DrivetrainControls();

    private Telemetry telemetry = null;

    public Drivetrain(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        init(hardwareMap);
    }

    public void init(HardwareMap hardwareMap){
        frontLeftDrive = hardwareMap.get(DcMotor.class, "lfdrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rfdrive");
        rearLeftDrive = hardwareMap.get(DcMotor.class, "lbdrive");
        rearRightDrive = hardwareMap.get(DcMotor.class, "rbdrive");
        driveMotors[0] = frontLeftDrive;
        driveMotors[1] = frontRightDrive;
        driveMotors[2] = rearRightDrive;
        driveMotors[3] = rearLeftDrive;
        frontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        applyAction(driveMotors, (motor) -> motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER));
        // for now, to be changed
        applyAction(driveMotors, (motor) -> motor.setZeroPowerBehavior(BRAKE));
        applyAction(driveMotors, (motor) -> motor.setPower(0.0));
    }

    public void setPowers(double [] powers) {
        if (powers != null && powers.length == 4)
            targetPowers = powers;
    }

    public void run(boolean autoMode){
        // For now we only implemented non-auto mode.
        setPowers(computeDriveTrainPower(this.controls));
        this.telemetry.addData("dr pwr", this.targetPowers);
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
        double frontLeftPower  = controls.translationYPower - controls.translationXPower + controls.rotationPower;
        double frontRightPower = controls.translationYPower + controls.translationXPower - controls.rotationPower;
        double rearLeftPower   = controls.translationYPower + controls.translationXPower + controls.rotationPower;
        double rearRightPower  = controls.translationYPower - controls.translationXPower - controls.rotationPower;

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

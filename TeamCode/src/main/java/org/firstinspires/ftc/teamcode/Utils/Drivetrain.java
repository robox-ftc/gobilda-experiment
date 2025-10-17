package org.firstinspires.ftc.teamcode.Utils;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static org.firstinspires.ftc.teamcode.Utils.Utils.applyAction;
import static org.firstinspires.ftc.teamcode.Utils.Utils.applyActions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {

    // Declare OpMode members.
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor rearLeftDrive = null;
    private DcMotor rearRightDrive = null;
    private DcMotor[] driveMotors = new DcMotor[4];
    private double[] targetPowers = new double[4];

    public Drivetrain(HardwareMap hardwareMap){
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
        applyAction(driveMotors, (motor) -> motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER));
        // for now, to be changed
        applyAction(driveMotors, (motor) -> motor.setZeroPowerBehavior(BRAKE));
        applyAction(driveMotors, (motor) -> motor.setPower(0.0));
    }

    public void setPowers(double [] powers) {
        if (powers != null && powers.length == 4)
            targetPowers = powers;
    }

    public void run(){
        applyActions(driveMotors, (motor, i) -> motor.setPower(this.targetPowers[i]));
    }
}

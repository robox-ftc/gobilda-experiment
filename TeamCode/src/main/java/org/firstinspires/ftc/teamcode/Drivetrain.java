package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static org.firstinspires.ftc.teamcode.Utils.applyAction;
import static org.firstinspires.ftc.teamcode.Utils.applyActions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {

    // Declare OpMode members.
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor rearLeftDrive = null;
    private DcMotor rearRightDrive = null;
    private DcMotor[] driveMotors = new DcMotor[4];

    public Drivetrain(HardwareMap harwareMap){
        init(harwareMap);
    }

    public void init(HardwareMap hardwareMap){

        frontLeftDrive = hardwareMap.get(DcMotor.class, "front-left-drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front-right-drive");
        rearLeftDrive = hardwareMap.get(DcMotor.class, "rear-left-drive");
        rearRightDrive = hardwareMap.get(DcMotor.class, "rear-right-drive");

        driveMotors[0] = frontLeftDrive;
        driveMotors[1] = frontRightDrive;
        driveMotors[2] =rearRightDrive;
        driveMotors[3] =  rearLeftDrive;

        applyAction(driveMotors, (motor) -> motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER));
        // for now, to be changed
        applyAction(driveMotors, (motor) -> motor.setZeroPowerBehavior(BRAKE));

        /*
         * set Feeders to an initial value to initialize the servo controller
         */
        applyAction(driveMotors, (motor) -> motor.setPower(0.0));
    }

    public void setPowers(double [] powers){
        applyActions(driveMotors, (motor, i) -> motor.setPower(powers[i]));
    }
}

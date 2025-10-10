package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static org.firstinspires.ftc.teamcode.Utils.applyAction;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    public static double INTAKE_MAX_VELOCITY = 314;
    public static double INTAKE_MIN_VELOCITY = 117;


    private DcMotorEx frontIntakeWheel = null;
    private double targetPower = 0.0;

    public Intake(HardwareMap hardwareMap) {
        frontIntakeWheel = hardwareMap.get(DcMotorEx.class, "intake");
        frontIntakeWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontIntakeWheel.setZeroPowerBehavior(BRAKE);
    }

    public void setPower(double power){
        targetPower = power;
    }

    public void spin() {
        frontIntakeWheel.setPower(targetPower);
    }

    public void stop(){
        targetPower = 0.0;
        frontIntakeWheel.setPower(targetPower);
    }
}

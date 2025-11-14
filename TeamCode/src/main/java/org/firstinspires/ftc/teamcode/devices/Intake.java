package org.firstinspires.ftc.teamcode.devices;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    private DcMotorEx frontIntakeWheel = null;
    private Telemetry telemetry;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        frontIntakeWheel = hardwareMap.get(DcMotorEx.class, "intake");
        frontIntakeWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontIntakeWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        frontIntakeWheel.setZeroPowerBehavior(BRAKE);
        this.telemetry = telemetry;
    }

    public void spin(double targetPower) {
        frontIntakeWheel.setPower(targetPower);
    }

    public boolean isBallDetected(){
        return false;
    }
    public boolean isChamberFull(){
        return true;
    }

    public void run(double manualPower){
        double autoPower = 0.0;
        if (isBallDetected() && !isChamberFull())
                autoPower = 1.0;
        double targetPower = Math.max(-1.0, Math.min(1.0, manualPower + autoPower));
        spin(targetPower);
    }
}
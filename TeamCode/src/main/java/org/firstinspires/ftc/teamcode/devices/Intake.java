package org.firstinspires.ftc.teamcode.devices;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    private DcMotorEx wheel = null;
    private Telemetry telemetry;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        wheel = hardwareMap.get(DcMotorEx.class, "intake");
        wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel.setDirection(DcMotorSimple.Direction.REVERSE);
        wheel.setZeroPowerBehavior(FLOAT);
        this.telemetry = telemetry;
    }

    public void spin(double targetPower) {
        wheel.setPower(targetPower);
    }


    public void run(double manualPower){
        double autoPower = 1.0;
        double targetPower = Math.max(-1.0, Math.min(1.0, manualPower + autoPower));
        spin(targetPower);
    }
}
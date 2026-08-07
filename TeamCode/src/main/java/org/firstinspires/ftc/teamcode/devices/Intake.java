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
        // FORWARD so that positive power pulls a ball in and negative ejects it.
        // Every caller (NewTeleop, NewAuto, AutoPractice) assumes spin(+1) = intake.
        wheel.setDirection(DcMotorSimple.Direction.FORWARD);
        wheel.setZeroPowerBehavior(FLOAT);
        this.telemetry = telemetry;
    }

    public void spin(double targetPower) {
        wheel.setPower(targetPower);
    }


    public void run(double manualPower) { // changed, formula was flawed
        double targetPower = Math.max(-1.0, Math.min(1.0, manualPower)); // clamp
        spin(targetPower);
    }
}
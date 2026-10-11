package org.firstinspires.ftc.teamcode.devices;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Ramp {
    private DcMotorEx wheel;
    private Telemetry telemetry;

    public Ramp(HardwareMap hardwareMap, Telemetry telemetry) {
        wheel = hardwareMap.get(DcMotorEx.class, "ramp");
        wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheel.setDirection(DcMotorSimple.Direction.FORWARD);
        wheel.setZeroPowerBehavior(FLOAT);
        this.telemetry = telemetry;
    }

    public void spin(double targetPower) {
        wheel.setPower(targetPower);
    }


    public void run(LauncherControls controls) {
        spin(controls.rampPower);
    }
}
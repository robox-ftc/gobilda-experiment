package org.firstinspires.ftc.teamcode.devices;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Hood {
    private static CRServoImplEx wheel = null;
    private Telemetry telemetry;

    public Hood(HardwareMap hardwareMap, Telemetry telemetry) {
        wheel = hardwareMap.get(CRServoImplEx.class, "hood");
        wheel.setDirection(CRServo.Direction.FORWARD);
        this.telemetry = telemetry;
    }

    public static void spin(double targetPower) {
        wheel.setPower(targetPower);
    }


    public static void run(double manualPower) { // changed, formula was flawed
        double targetPower = Math.max(-1.0, Math.min(1.0, manualPower)); // clamp
        spin(targetPower);
    }
}
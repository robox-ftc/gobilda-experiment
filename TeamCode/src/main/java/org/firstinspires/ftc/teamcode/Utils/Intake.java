package org.firstinspires.ftc.teamcode.Utils;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake implements IDevice {
    private DcMotorEx frontIntakeWheel = null;
    private double manualPower = 0.0;
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
        return true;
    }

    public boolean isChamberFull(){
        return false;
    }

    public void run(boolean autoMode){
        double targetPower = 0.0;
        if (autoMode){
            if (isBallDetected() && !isChamberFull())
                targetPower = 1.0;
        }
        else{
            targetPower = manualPower;
        }

        spin(targetPower);
    }

    public void readControls(GamePadReadings oldReadings, GamePadReadings newReadings) {
        this.manualPower = newReadings.rightBumper ? 1.0 : newReadings.rightTrigger;

        telemetry.addData("intake", this.manualPower);
    }
}

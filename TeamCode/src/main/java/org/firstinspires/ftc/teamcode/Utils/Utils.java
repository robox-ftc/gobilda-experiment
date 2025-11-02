package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.function.BiConsumer;
import java.util.function.Consumer;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Utils {

    public static <T> void applyAction(T[] objs, Consumer<T> action) {
        for (T obj : objs) {
            action.accept(obj);
        }
    }

    public static <T> void applyActions(T[] objs, BiConsumer<T, Integer> action) {
        for (int i = 0; i < objs.length; i++) {
            action.accept(objs[i], i);
        }
    }
/*
    public boolean homeToStop(DcMotorEx motor, double homingPower, double timeoutSec,
                              double currentThreshold, int stallTickTolerance, double stallTimeThreshold) {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(homingPower);

        int lastPos = motor.getCurrentPosition();
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime stillTimer = new ElapsedTime();

        while (timer.seconds() < timeoutSec) {
            int pos = motor.getCurrentPosition();
            int delta = pos - lastPos;
            double current = motor.getCurrent(CurrentUnit.AMPS);
            boolean noMotion = Math.abs(delta) < stallTickTolerance;
            boolean highCurrent = current > currentThreshold;

            if (noMotion) {
                // started being still
                if (stillTimer.seconds() == 0)
                    stillTimer.reset();

                // stayed still long enough, or current surge seen
                if (stillTimer.seconds() > stallTimeThreshold || highCurrent) {
                    stopAndResetEncoder();
                    return true;
                }
            } else {
                stillTimer.reset();
            }

            lastPos = pos;

            try {
                Thread.sleep(20); // small delay to reduce CPU load
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        motor.setPower(0);
        return false; // timeout, failed to home
    }
*/
    public static void stopAndResetEncoder(DcMotorEx motor) {
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public static boolean buttonUp(boolean oldButtonState, boolean newButtonState){
        // When the mode button is "up", we toggle the mode between auto and manual.
        return oldButtonState && !newButtonState;
    }

    public static boolean toggle(boolean oldState, boolean flip){
        return flip ? !oldState : oldState;
    }
}

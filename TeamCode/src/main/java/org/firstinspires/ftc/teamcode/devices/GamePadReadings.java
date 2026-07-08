package org.firstinspires.ftc.teamcode.devices;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamePadReadings {
    public double leftStickX;
    public double leftStickY;
    public double rightStickX;
    public double rightStickY;

    public boolean leftBumper;
    public boolean rightBumper;

    public boolean aButton;
    public boolean bButton;
    public boolean xButton;
    public boolean yButton;

    public boolean dPadUp;
    public boolean dPadDown;
    public boolean dPadLeft;
    public boolean dPadRight;
    public boolean dPad;
    public double leftTrigger;
    public double rightTrigger;

    public boolean aWasReleased;
    public boolean aWasPressed;
    public boolean bWasReleased;
    public boolean bWasPressed;
    public boolean xWasReleased;
    public boolean xWasPressed;
    public boolean yWasReleased;
    public boolean yWasPressed;
    public boolean backWasPressed;

    public void update(Gamepad gamepad1, Gamepad gamepad2) {
        bButton = gamepad1.b || gamepad2.b;
        aButton = gamepad1.a || gamepad2.a;
        xButton = gamepad1.x || gamepad2.x;
        yButton = gamepad1.y || gamepad2.y;
        leftBumper = gamepad1.left_bumper || gamepad2.left_bumper;
        rightBumper = gamepad1.right_bumper || gamepad2.right_bumper;

        leftStickX =  Math.abs(gamepad1.left_stick_x) >= Math.abs(gamepad2.left_stick_x) ?
                gamepad1.left_stick_x : gamepad2.left_stick_x;
        leftStickY = Math.abs(gamepad1.left_stick_y) >= Math.abs(gamepad2.left_stick_y) ?
                gamepad1.left_stick_y : gamepad2.left_stick_y;
        rightStickX = Math.abs(gamepad1.right_stick_x) >= Math.abs(gamepad2.right_stick_x) ?
                gamepad1.right_stick_x : gamepad2.right_stick_x;
        rightStickY =  Math.abs(gamepad1.right_stick_y) >= Math.abs(gamepad2.right_stick_y) ?
                gamepad1.right_stick_y : gamepad2.right_stick_y;


        leftTrigger = Math.max(gamepad1.left_trigger, gamepad2.left_trigger);
        rightTrigger = Math.max(gamepad1.right_trigger, gamepad2.right_trigger);

        dPadUp = gamepad1.dpad_up || gamepad2.dpad_up;
        dPadDown = gamepad1.dpad_down || gamepad2.dpad_down;
        dPadLeft = gamepad1.dpad_left || gamepad2.dpad_left;
        dPadRight = gamepad1.dpad_right || gamepad2.dpad_right;
        dPad = dPadUp || dPadDown || dPadLeft || dPadRight;

        aWasReleased = gamepad1.aWasReleased() || gamepad2.aWasReleased();
        aWasPressed = gamepad1.aWasPressed() || gamepad2.aWasPressed();

        bWasReleased = gamepad1.bWasReleased() || gamepad2.bWasReleased();
        bWasPressed = gamepad1.bWasPressed() || gamepad2.bWasPressed();

        xWasReleased = gamepad1.xWasReleased() || gamepad2.xWasReleased();
        xWasPressed = gamepad1.xWasPressed() || gamepad2.xWasPressed();

        yWasReleased = gamepad1.yWasReleased() || gamepad2.yWasReleased();
        yWasPressed = gamepad1.yWasPressed() || gamepad2.yWasPressed();
        backWasPressed = gamepad1.backWasPressed() || gamepad2.backWasPressed();
    }
}

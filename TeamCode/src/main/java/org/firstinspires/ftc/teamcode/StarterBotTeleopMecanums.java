/*
 * Copyright (c) 2025 FIRST
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Utils.applyAction;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

/*
 * This file includes a teleop (driver-controlled) file for the goBILDA® StarterBot for the
 * 2025-2026 FIRST® Tech Challenge season DECODE™. It leverages a differential/Skid-Steer
 * system for robot mobility, one high-speed motor driving two "launcher wheels", and two servos
 * which feed that launcher.
 *
 * Likely the most niche concept we'll use in this example is closed-loop motor velocity control.
 * This control method reads the current speed as reported by the motor's encoder and applies a varying
 * amount of power to reach, and then hold a target velocity. The FTC SDK calls this control method
 * "RUN_USING_ENCODER". This contrasts to the default "RUN_WITHOUT_ENCODER" where you control the power
 * applied to the motor directly.
 * Since the dynamics of a launcher wheel system varies greatly from those of most other FTC mechanisms,
 * we will also need to adjust the "PIDF" coefficients with some that are a better fit for our application.
 */

@TeleOp(name = "StarterBotTeleopMecanums", group = "StarterBot")
// @Disabled
public class StarterBotTeleopMecanums extends StarterBotAuto {
    final double STOP_SPEED = 0.0; // We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;

    // Setup a variable for each drive wheel to save power level for telemetry
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        super.init();
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TIME_BETWEEN_SHOTS = 0;
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        super.init_loop();
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        /*
         * Here we call a function called arcadeDrive. The arcadeDrive function takes
         * the input from
         * the joysticks, and applies power to the left and right drive motor to move
         * the robot
         * as requested by the driver. "arcade" refers to the control style we're using
         * here.
         * Much like a classic arcade game, when you move the left joystick forward both
         * motors
         * work to drive the robot forward, and when you move the right joystick left
         * and right
         * both motors work to rotate the robot. Combinations of these inputs can be
         * used to create
         * more complex maneuvers.
         */
        String data = "";
        if (odo != null) {
            odo.update();
            Pose2D pos = odo.getPosition();
            data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM),
                    pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
        }

        targetTag = locateTarget(targetTagId);

        mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x,
                reverseRotate ? -gamepad1.right_stick_x : gamepad1.right_stick_x);

        /*
         * Here we give the user control of the speed of the launcher motor without
         * automatically
         * queuing a shot.
         */
        if (gamepad1.y) {
            if (!drivetrainOnly) {
                applyAction(launchers, (launcher) -> launcher.setVelocity(LAUNCHER_TARGET_VELOCITY_2));
            }
        } else if (gamepad1.b) { // stop flywheel
            if (!drivetrainOnly) {
                applyAction(launchers, (launcher) -> launcher.setVelocity(STOP_SPEED));
            }
        }

        if (gamepad1.start) {
            aim(1);
        }

        /*
         * Now we call our "Launch" function.
         */
        if (!drivetrainOnly) {
            launch(gamepad1.rightBumperWasPressed(), LAUNCHER_TARGET_VELOCITY_2);
            launch(gamepad1.xWasPressed(), LAUNCHER_TARGET_VELOCITY_1);
            launch(gamepad1.aWasPressed(), LAUNCHER_TARGET_VELOCITY_3);

            frontIntakeWheel.setPower(-gamepad1.left_trigger);
            if (gamepad1.dpad_up || gamepad1.dpad_down) {
                int turretDegree = (int) TURRET_TICKS_PER_DEGREE;
                int currentPosition = turret.getCurrentPosition();
                turret.setTargetPosition(currentPosition + (gamepad1.dpad_up ? turretDegree : -turretDegree));
                // turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                turret.setPower(gamepad1.dpad_up ? 1 : -1);
            } else {
                turret.setPower(0);
            }
        }

        /*
         * Show the state and motor powers
         */
        telemetry.addData("Team", alliance);
        telemetry.addData("LauncherState", launchState);
        if (targetTag != null) {
            telemetry.addData("Tag Location",
                    String.format(Locale.US, "{X: %.3f, Y: %.3f}", targetTag.ftcPose.x, targetTag.ftcPose.y));
        }
        if (!drivetrainOnly) {
            telemetry.addData("motorSpeed", "left (%.0f), right (%.0f)", launchers[0].getVelocity(),
                    launchers[1].getVelocity());
        }
        telemetry.addData("Motor Current Positions", "left (%d), right (%d)",
                leftFrontDrive.getCurrentPosition(),
                rightFrontDrive.getCurrentPosition());
        telemetry.addData("Motor Target Positions", "left (%d), right (%d)",
                leftFrontDrive.getTargetPosition(),
                rightFrontDrive.getTargetPosition());
        telemetry.addData("Position", data);
        if (portal != null) {
            telemetry.addData("Camera Status", portal.getCameraState());
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
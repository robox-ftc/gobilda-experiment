/* Copyright (c) 2017 FIRST. All rights reserved.
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
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Mat;


/// Rename OpMode here
@TeleOp(name="Robot Demo", group="Iterative OpMode")
public class BasicOpMode_Iterative extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor armMotor = null;
    private Servo servoThing = null;
    private Servo intakeWheel = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        ///  Get the motors
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        servoThing = hardwareMap.get(Servo.class, "intake_wheel");
        intakeWheel = hardwareMap.get(Servo.class, "intake");

        /// idk what this does but is probably important
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * This runs coninuously when you press Start and stops when you press Stop
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        // The right joystick controls the drivetrain
        // and the left one controls the motors on the arm

        /// Save the value as a variable
        double x = gamepad1.right_stick_x;
        double y = gamepad1.right_stick_y;

        /// Absolute value for calculation purposes
        double xa = Math.abs(x);
        double ya = Math.abs(y);

        // Set the speed of how fast the robot can move
        double speed = 0.5;

        /// Conditionals that set the power to the motors
        /// depending on what buttons are being pressed

        // Rotate to the right
        if (x < 0) {
            leftDrive.setPower(speed * xa);
            rightDrive.setPower(-speed * xa);
        }

        // Rotate to the left
        else if (x > 0) {
            rightDrive.setPower(speed * xa);
            leftDrive.setPower(-speed * xa);
        }
        else {
            // Move forward
            if (y > 0) {
                rightDrive.setPower(speed * ya);
                leftDrive.setPower(speed * ya);
            }

            // Move backward
            else if (y < 0) {
                rightDrive.setPower(-speed * ya);
                leftDrive.setPower(-speed * ya);
            } else {
                // If no buttons are being pressed
                // on the right joystick,
                // Set the motor powers to zero to stop movement
                rightDrive.setPower(0);
                leftDrive.setPower(0);
            }
        }

        // arm thingy
        double armSpeed = 0.25;
        double armY = gamepad1.left_stick_y;
        armMotor.setPower(armSpeed * armY);

        //inake wheel thing
        double servoSpeed = 0.25;
        double servoX = gamepad1.left_stick_x;
        servoThing.setPosition((-servoX + 1)/ 2);

        // TODO: the actual wheel
        // TODO: get the triggers working
        // Get trigger values
        double right = gamepad1.right_trigger;
        double left = gamepad1.left_trigger;

// Default position is 0.5 (stopped)
        double pos = 0.5;

// If right trigger is pressed, move forward (0.5 to 1.0)
        if (right > 0.1) {
            pos = 0.5 + (right * 0.5);
        }
// If left trigger is pressed, move in reverse (0.5 to 0.0)
        else if (left > 0.1) {
            pos = 0.5 - (left * 0.5);
        }

// Set the servo's position once with the final calculated value
        intakeWheel.setPosition(pos);
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower)
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    public void waitMS(int mills) {
        try {
            Thread.sleep(mills);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}

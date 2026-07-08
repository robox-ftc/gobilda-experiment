
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.devices.*;

@TeleOp(name = "Teleop-2026Decode", group = "StarterBot")
public class NewTeleop extends OpMode {

    public enum StartPosition {
        NEAR,
        FAR
    }
    private GamePadReadings gamePadReading;
    private Launcher launcher;
    private Drivetrain drivetrain;
    private Intake intake;
    private Turret turret;
    private int targetTagId = 20;
    private StartPosition startPosition = StartPosition.NEAR;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        gamePadReading = new GamePadReadings();
        drivetrain = new Drivetrain(hardwareMap, telemetry, false);
        launcher = new Launcher(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        telemetry.addData("Status", "Motors Initialized");
        turret = new Turret(hardwareMap, telemetry);
        telemetry.addLine("Camera initialized. Waiting for start...");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        // choose color and location, can be changed if misclicked, default blue near
        if (gamepad1.xWasPressed() || gamepad2.xWasPressed()) {
            targetTagId = 20;
            telemetry.addLine("Team BLUE is selected");
        }
        if (gamepad1.bWasPressed() || gamepad2.bWasPressed()){
            targetTagId = 24;
            telemetry.addLine("Team RED is selected.");
        }
        if (gamepad1.yWasPressed() || gamepad2.yWasPressed()) {
            startPosition = StartPosition.FAR;
            telemetry.addLine("Starting position FAR.");
        }
        if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
            startPosition = StartPosition.NEAR;
            telemetry.addLine("Starting position NEAR.");
        }
        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        telemetry.addData("Target ID", targetTagId);
        telemetry.addData("Starting from", startPosition);
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        // Sensing
        // This is the global readings.
        gamePadReading.update(gamepad1, gamepad2);

        DrivetrainControls driveControls = DrivetrainControls.readControls(gamePadReading);
        LauncherControls launcherControls = LauncherControls.readControls(gamePadReading);
        double intakePower = gamePadReading.rightBumper ? 1.0 : Math.max(gamePadReading.rightTrigger, gamePadReading.leftTrigger);
        intake.run(intakePower);
        turret.run(targetTagId, gamePadReading);

        if (gamePadReading.bWasReleased) {
            launcher.abort();
        } else launcher.run(launcherControls);

        drivetrain.run(driveControls);
        telemetry.update();
    }
}
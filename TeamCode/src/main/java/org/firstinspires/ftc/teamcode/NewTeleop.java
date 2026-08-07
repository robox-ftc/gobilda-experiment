
package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
    private GoBildaPinpointDriver pinpoint;
    private Turret turret;
    private int targetTagId = 20;
    private final double TOLERANCE = 0.1;
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
        telemetry.addData("Status: ", "Motors Initialized");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD); // Not necessarily correct
        pinpoint.setOffsets(0, 7.5, DistanceUnit.INCH);
        pinpoint.resetPosAndIMU();
        telemetry.addData("Status: ", "Odo Initialized");
        try {
            turret = new Turret(hardwareMap, telemetry);
            telemetry.addLine("Camera initialized. Waiting for start...");
        } catch (Exception e) {
            // A missing webcam must not cost us the whole TeleOp - drive and
            // shoot still work, only auto-aim is gone.
            turret = null;
            telemetry.addLine("NO TURRET/CAMERA - driving and launching still available");
        }
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
        pinpoint.update();
        double heading = pinpoint.getHeading(AngleUnit.DEGREES);

        DrivetrainControls driveControls = DrivetrainControls.readControls(gamePadReading);
        LauncherControls launcherControls = LauncherControls.readControls(gamePadReading);

        /*
         * The intake has to push the ball into the feeder for a shot to happen, so
         * firing takes it over. B stays the "eject everything" button: the launcher
         * wheels already reverse on B, and the intake now reverses with them.
         * Otherwise the triggers run it in.
         */
        double intakePower;
        if (launcherControls.triggerDown) {
            intakePower = 1.0;
        } else if (gamePadReading.bButton) {
            intakePower = -1.0;
        } else {
            intakePower = Math.max(gamePadReading.rightTrigger, gamePadReading.leftTrigger);
        }
        intake.spin(intakePower);

        if (turret != null) {
            turret.run(targetTagId, heading, gamePadReading);
        }
        launcher.run(launcherControls);
        telemetry.addData("Intake", "%.2f", intakePower);

        drivetrain.run(driveControls);
        telemetry.addData("X", pinpoint.getPosX(DistanceUnit.INCH));
        telemetry.addData("Y", pinpoint.getPosY(DistanceUnit.INCH));
        telemetry.addData("Heading", heading);
        telemetry.update();
    }
}
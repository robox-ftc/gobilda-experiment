
package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.devices.*;

@TeleOp(name = "Teleop-2026Decode", group = "StarterBot")
public class NewTeleop extends OpMode {
    private GamePadReadings gamePadReading;
    private Launcher launcher;
    private Drivetrain drivetrain;
    private Intake intake;
    private Ramp ramp;
    private GoBildaPinpointDriver pinpoint;
    private final double TOLERANCE = 0.1;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        gamePadReading = new GamePadReadings();
        drivetrain = new Drivetrain(hardwareMap, telemetry, false);
        launcher = new Launcher(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        ramp = new Ramp(hardwareMap, telemetry);
        telemetry.addData("Status: ", "Motors Initialized");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD); // Not necessarily correct
        pinpoint.setOffsets(0, 7.5, DistanceUnit.INCH);
        pinpoint.resetPosAndIMU();
        telemetry.addData("Status: ", "Odo Initialized");
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
            telemetry.addLine("Team BLUE is selected");
        }
        if (gamepad1.bWasPressed() || gamepad2.bWasPressed()){
            telemetry.addLine("Team RED is selected.");
        }
        telemetry.update();
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
        // Sensing
        // This is the global readings.
        gamePadReading.update(gamepad1, gamepad2);
        pinpoint.update();
        double heading = pinpoint.getHeading(AngleUnit.DEGREES);

        DrivetrainControls driveControls = DrivetrainControls.readControls(gamePadReading);
        LauncherControls launcherControls = LauncherControls.readControls(gamePadReading);
        double intakePower = gamePadReading.bButton ? -0.5 : gamePadReading.rightTrigger;
        intake.run(intakePower);
        launcher.run(launcherControls);
        ramp.run(launcherControls);

        drivetrain.run(driveControls);
        telemetry.addData("X", pinpoint.getPosX(DistanceUnit.INCH));
        telemetry.addData("Y", pinpoint.getPosY(DistanceUnit.INCH));
        telemetry.addData("Heading", heading);
        telemetry.update();
    }
}
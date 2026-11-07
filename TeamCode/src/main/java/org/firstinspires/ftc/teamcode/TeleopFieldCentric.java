package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.devices.*;

@TeleOp(name = "Teleop-2027-Field-Centric", group = "StarterBot")
public class TeleopFieldCentric extends OpMode {

    private GamePadReadings gamePadReading;
    private Launcher launcher;
    private Drivetrain drivetrain;
    private Intake intake;
    private Ramp ramp;

    private GoBildaPinpointDriver pinpoint;

    private double headingOffset = 0.0;
    private boolean previousOptions = false;

    private final double TOLERANCE = 0.1;

    @Override
    public void init() {

        gamePadReading = new GamePadReadings();

        drivetrain = new Drivetrain(hardwareMap, telemetry, false);
        launcher = new Launcher(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        ramp = new Ramp(hardwareMap, telemetry);

        // Initialize Pinpoint
        pinpoint = hardwareMap.get(
                GoBildaPinpointDriver.class, "pinpoint"
        );

        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );

        pinpoint.setOffsets(0, 7.5, DistanceUnit.INCH);

        pinpoint.resetPosAndIMU();

        telemetry.addLine("Motors Initialized");
        telemetry.addLine("Pinpoint Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {

        if (gamepad1.xWasPressed() || gamepad2.xWasPressed()) {
            telemetry.addLine("Team BLUE is selected");
        }

        if (gamepad1.bWasPressed() || gamepad2.bWasPressed()) {
            telemetry.addLine("Team RED is selected");
        }

        telemetry.update();
    }

    @Override
    public void start() {

        pinpoint.update();

        // Save starting heading
        headingOffset = pinpoint.getHeading(AngleUnit.RADIANS);
    }

    // Convert robot-centric joystick inputs
    // into field-centric joystick inputs
    private DrivetrainControls makeFieldCentric(
            DrivetrainControls controls,
            double heading) {

        double x = controls.translationXPower;
        double y = controls.translationYPower;

        double rotX = x * Math.cos(-heading)
                - y * Math.sin(-heading);

        double rotY = x * Math.sin(-heading)
                + y * Math.cos(-heading);

        return new DrivetrainControls(
                rotX,
                rotY,
                controls.rotationPower
        );
    }

    @Override
    public void loop() {

        // Update gamepad
        gamePadReading.update(gamepad1, gamepad2);
        if (gamePadReading.backWasPressed) {
            pinpoint.resetPosAndIMU();
        }

        // Update Pinpoint
        pinpoint.update();

        // Get heading
        double heading =
                pinpoint.getHeading(AngleUnit.RADIANS);

        // Reset field-centric direction with OPTIONS
        boolean optionsNow = gamepad1.options;

        if (optionsNow && !previousOptions) {
            headingOffset = heading;
        }

        previousOptions = optionsNow;

        // Calculate adjusted heading
        double adjustedHeading = heading - headingOffset;

        // Read joystick controls
        DrivetrainControls driveControls =
                DrivetrainControls.readControls(gamePadReading);

        // Convert to field-centric
        DrivetrainControls fieldControls =
                makeFieldCentric(driveControls, adjustedHeading);

        // Launcher controls
        LauncherControls launcherControls =
                LauncherControls.readControls(gamePadReading);

        // Intake
        double intakePower =
                gamePadReading.bButton
                        ? -0.5
                        : gamePadReading.rightTrigger;

        intake.run(intakePower);

        // Launcher and ramp
        launcher.run(launcherControls);
        ramp.run(launcherControls);

        // Field-centric drivetrain
        drivetrain.run(fieldControls);

        // Telemetry
        telemetry.addData("Heading", heading);
        telemetry.addData("Adjusted Heading", adjustedHeading);
        telemetry.addData("Field X", fieldControls.translationXPower);
        telemetry.addData("Field Y", fieldControls.translationYPower);

        telemetry.update();
    }
}
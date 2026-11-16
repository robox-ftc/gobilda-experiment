package org.firstinspires.ftc.teamcode.testing;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.TeleOp;
import org.junit.Test;

/**
 * START on the pad starts the OpMode in the harness, but on the robot START is
 * a Driver Station screen button, so the OpMode must not see that press.
 * Teleop-2027-Intel flips the left stick on every START it does see.
 */
public class PadStartTest {

    private static final double DT = 0.02;

    @Test
    public void theStartThatLaunchesTheOpModeIsHiddenUntilLetGo() {
        DriverStation ds = new DriverStation();
        ds.held1.start = true;
        ds.takeStart1();
        ds.copyIn();
        assertFalse(ds.gamepad1.start);
        assertFalse(ds.gamepad1.startWasPressed());

        ds.held1.start = false;
        ds.copyIn();
        ds.held1.start = true;
        ds.copyIn();
        assertTrue("a later press reaches the OpMode", ds.gamepad1.start);
        assertTrue(ds.gamepad1.startWasPressed());
    }

    @Test
    public void otherButtonsStillReachTheOpModeWhileStartIsHidden() {
        DriverStation ds = new DriverStation();
        ds.held1.start = true;
        ds.takeStart1();
        ds.held1.a = true;
        ds.held1.left_stick_x = 0.5f;
        ds.copyIn();
        assertTrue(ds.gamepad1.a);
        assertEquals(0.5f, ds.gamepad1.left_stick_x, 1e-6);
    }

    /** Front-left power while the left stick is held forward. */
    private double forward(OpModeRunner runner, DesktopRobot robot, DriverStation ds) {
        ds.held1.left_stick_y = -1f;
        for (int i = 0; i < 3; i++) runner.cycle(DT);
        double fl = robot.frontLeft.power;
        ds.held1.left_stick_y = 0f;
        runner.cycle(DT);
        return fl;
    }

    @Test
    public void startingTeleop2027IntelFromThePadDoesNotReverseTheStick() {
        DesktopRobot plain = new DesktopRobot();
        DriverStation plainDs = new DriverStation();
        double unreversed;
        try (OpModeRunner runner = new OpModeRunner(TeleOp.class, plain, plainDs)) {
            runner.start(); // the screen button
            runner.cycle(DT);
            unreversed = forward(runner, plain, plainDs);
        }

        DesktopRobot robot = new DesktopRobot();
        DriverStation ds = new DriverStation();
        try (OpModeRunner runner = new OpModeRunner(TeleOp.class, robot, ds)) {
            runner.cycle(DT);
            ds.held1.start = true; // START on the pad, as the harness loop sees it
            ds.takeStart1();
            runner.start();
            for (int i = 0; i < 3; i++) runner.cycle(DT);
            ds.held1.start = false;
            runner.cycle(DT);
            assertEquals("launching from the pad", unreversed, forward(runner, robot, ds), 1e-9);

            // A START while it runs is the driver's toggle, and still flips it.
            ds.held1.start = true;
            runner.cycle(DT);
            ds.held1.start = false;
            runner.cycle(DT);
            assertEquals("the driver's toggle", -unreversed, forward(runner, robot, ds), 1e-9);
        }
    }

    @Test
    public void aStickRestingSlightlyOffCentreReadsAsZero() {
        assertEquals(0f, GamepadHarness.deadzoned(0.0665f), 0f);
        assertEquals(0f, GamepadHarness.deadzoned(-0.0665f), 0f);
        assertEquals(0.3f, GamepadHarness.deadzoned(0.3f), 0f);
    }
}

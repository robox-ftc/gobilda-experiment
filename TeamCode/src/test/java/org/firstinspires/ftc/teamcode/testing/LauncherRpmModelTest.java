package org.firstinspires.ftc.teamcode.testing;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.devices.Launcher;

import org.junit.Test;

/**
 * The launcher wheels are the one piece of hardware the harness models rather
 * than merely records. These drive the real Launcher, built from the desktop
 * hardware map, and read back what it would read back.
 *
 * <p>They assert the <em>shape</em> of the model, never a number off the
 * guessed constants: nobody has measured this flywheel.
 */
public class LauncherRpmModelTest {

    private final DesktopRobot robot = new DesktopRobot();
    private final Launcher launcher = new Launcher(robot.hardwareMap, robot.telemetry.device);

    private void steps(int n, double dt) {
        for (int i = 0; i < n; i++) robot.step(dt);
    }

    @Test
    public void aMotorWithNoPlantStillReportsTheCommandedVelocity() {
        robot.intakeWheel.device.setVelocity(500);

        assertEquals(500.0, robot.intakeWheel.device.getVelocity(), 1e-9);
        assertEquals(0.0, robot.intakeWheel.simTicksPerSec, 1e-9);
    }

    @Test
    public void theReadbackIsNotTheCommand() {
        launcher.spin(0.75);

        assertEquals("the command", 0.75 * DesktopRobot.LAUNCHER_FREE_RPM,
                robot.commandedWheelRPM(), 1e-6);
        assertEquals("what the code can read back", 0.0, launcher.getLeftWheelRPM(), 1e-9);
    }

    @Test
    public void theWheelsSpinUpTowardsTheCommandAndStopThere() {
        launcher.spin(0.75);

        double previous = 0;
        for (int i = 0; i < 400; i++) {
            robot.step(0.005);
            double rpm = launcher.getLeftWheelRPM();
            assertTrue("must not run backwards up the curve", rpm >= previous - 1e-9);
            assertTrue("must not overshoot the command", rpm <= robot.commandedWheelRPM() + 1e-9);
            previous = rpm;
        }

        assertEquals("two seconds is many time constants", robot.commandedWheelRPM(),
                launcher.getLeftWheelRPM(), robot.commandedWheelRPM() * 0.01);
        assertEquals("both wheels are driven together",
                launcher.getLeftWheelRPM(), launcher.getRightWheelRPM(), 1e-9);
        assertEquals("the harness reads it the way Launcher does",
                launcher.getLeftWheelRPM(), robot.leftWheelRPM(), 1e-9);
    }

    @Test
    public void oneTimeConstantIsAboutSixtyThreePercent() {
        launcher.spin(1.0);
        steps(1000, DesktopRobot.LAUNCHER_TAU_SECONDS / 1000);

        // 1 - 1/e. The property of the model, not of the guessed constants.
        assertEquals(0.632, robot.flywheelSpeed(), 0.01);
    }

    @Test
    public void cuttingThePowerLetsTheWheelsCoast() {
        // This Launcher is open loop and sets FLOAT, so spin(0) is a coast.
        launcher.spin(1.0);
        steps(400, 0.005);
        double spinning = robot.flywheelSpeed();

        launcher.spin(0);
        steps(200, 0.005);

        assertTrue("a floated flywheel does not stop dead", robot.flywheelSpeed() > spinning * 0.3);
        assertTrue("but it is losing speed", robot.flywheelSpeed() < spinning);
    }

    @Test
    public void aVelocityOfZeroIsHeldNotCoasted() {
        // spinToVelocity is the closed-loop path. setVelocity(0) leaves the
        // motor enabled with a target of zero (LynxDcMotorController, SDK
        // 11.0.0), so the velocity loop pulls the wheels down.
        launcher.spinToVelocity(DesktopRobot.LAUNCHER_PLANT.freeTicksPerSec);
        steps(400, 0.005);

        launcher.spinToVelocity(0);
        steps(400, 0.005);

        assertTrue(robot.flywheelSpeed() < 0.05);
    }
}

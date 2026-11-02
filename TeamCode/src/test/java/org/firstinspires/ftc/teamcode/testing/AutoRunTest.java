package org.firstinspires.ftc.teamcode.testing;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.BIOBUZZAuto;
import org.junit.After;
import org.junit.Test;

import java.util.function.Consumer;

/**
 * HiveKnightAuto ({@link BIOBUZZAuto}), the real class, run on stand-in motors.
 *
 * <p>Everything here goes through the OpMode as the robot controller calls it:
 * the mode is chosen with gamepad presses during INIT, START runs it, and the
 * motors, the dead-reckoned position and the telemetry are read back afterwards.
 * Time is the robot's simulated clock, so a 30-second autonomous runs in a few
 * milliseconds and the same way every time.
 *
 * <p>Some of these tests pin behaviour that is almost certainly not what was
 * meant. They say so where they do, and should be flipped when the auto is fixed.
 */
public class AutoRunTest {

    private static final double DT = 0.02; // 50 Hz, the harness's loop rate

    private final DriverStation ds = new DriverStation();
    private DesktopRobot robot = new DesktopRobot();
    private OpModeRunner runner = new OpModeRunner(BIOBUZZAuto.class, robot, ds);
    private final RobotSim sim = new RobotSim();

    @After
    public void closeRunner() {
        runner.close();
    }

    /** Rebuilds the robot with some devices missing from its configuration. */
    private void unplug(String... devices) {
        runner.close();
        robot = new DesktopRobot(devices);
        runner = new OpModeRunner(BIOBUZZAuto.class, robot, ds);
    }

    private void cycle() {
        runner.cycle(DT);
        sim.step(robot.frontLeft.power, robot.frontRight.power,
                robot.rearLeft.power, robot.rearRight.power, DT);
    }

    private void run(double seconds) {
        long n = Math.round(seconds / DT);
        for (long i = 0; i < n; i++) cycle();
    }

    /** One press and release of a button, as the OpMode sees it during INIT. */
    private void press(Consumer<Gamepad> button) {
        Gamepad held = new Gamepad();
        button.accept(held);
        ds.held1.copy(held);
        cycle();
        ds.releaseAll();
        cycle();
    }

    private static final Consumer<Gamepad> A = g -> g.a = true;
    private static final Consumer<Gamepad> B = g -> g.b = true;
    private static final Consumer<Gamepad> X = g -> g.x = true;
    private static final Consumer<Gamepad> Y = g -> g.y = true;
    private static final Consumer<Gamepad> RB = g -> g.right_bumper = true;

    // ---------------------------------------------------------------------- init

    @Test
    public void initReportsEachSubsystem() {
        assertEquals(OpModeRunner.Phase.INIT, runner.phase());
        assertTrue(robot.telemetry.visible().contains("Drivetrain initialized"));
        assertTrue(robot.telemetry.visible().contains("Launcher initialized"));
        assertTrue(robot.telemetry.visible().contains("Intake initialized"));
    }

    @Test
    public void aMissingIntakeFallsBackToDrivetrainOnly() {
        unplug("intake");

        assertTrue(robot.telemetry.visible().contains("Drivetrain-only started"));
    }

    @Test
    public void inDrivetrainOnlyModeTheFirstMoveCrashesTheAuto() {
        /*
         * Probably not intended. init() catches the missing intake and carries
         * on with intake = null, but a TRANSLATE task calls intake.spin(1)
         * without checking when it finishes. So the fallback the catch exists
         * for ends in a NullPointerException after the first move.
         */
        unplug("intake");
        runner.start();
        run(1.0);

        assertEquals(OpModeRunner.Phase.STOPPED, runner.phase());
        assertTrue("was " + runner.failure(), runner.failure() instanceof NullPointerException);
        assertEquals("it got as far as the end of the first move", 0.5, runner.runSeconds(), 0.05);
    }

    // --------------------------------------------------------------------- leave

    @Test
    public void withNothingChosenItLeavesForwardForHalfASecond() {
        runner.start();

        cycle();
        assertEquals(0.5, robot.frontLeft.power, 1e-9);
        assertEquals(-0.5, robot.frontRight.power, 1e-9);
        assertEquals(0.5, robot.rearLeft.power, 1e-9);
        assertEquals(-0.5, robot.rearRight.power, 1e-9);

        run(0.6);
        assertEquals("stopped after the half second", 0.0, robot.frontLeft.power, 1e-9);
        assertTrue("moved up the field, was " + sim.yInches, sim.yInches > RobotSim.FIELD_INCHES / 2 + 5);
        assertEquals("not sideways", RobotSim.FIELD_INCHES / 2, sim.xInches, 0.01);
        assertEquals("not turned", 0.0, sim.headingDeg, 0.01);
        assertNull(runner.failure());
    }

    @Test
    public void theIntakeRunsForOneCycleAfterAMove() {
        // The TRANSLATE task turns the intake on as it finishes, and the next
        // loop, with the queue empty, turns it straight off again: one cycle,
        // which is 20 ms here and one loop time on the robot.
        runner.start();
        int cyclesOn = 0;
        for (int i = 0; i < 100; i++) {
            cycle();
            if (robot.intakeFlow() > 0) cyclesOn++;
        }
        assertEquals(1, cyclesOn);
    }

    // ---------------------------------------------------------------------- park

    @Test
    public void parkFromTheHiveSideGoesForwardAndRight() {
        press(A); // Hive side
        press(B); // Park
        runner.start();
        run(1.5);

        double dx = sim.xInches - RobotSim.FIELD_INCHES / 2;
        double dy = sim.yInches - RobotSim.FIELD_INCHES / 2;
        assertTrue("right, was " + dx, dx > 1);
        assertTrue("forward, was " + dy, dy > 1);
        assertTrue("more forward than right (0.5 against 0.28)", dy > dx);
    }

    @Test
    public void parkFromTheWallSideGoesMoreRightThanForward() {
        press(B); // Wall side
        press(B); // Park
        runner.start();
        run(1.5);

        double dx = sim.xInches - RobotSim.FIELD_INCHES / 2;
        double dy = sim.yInches - RobotSim.FIELD_INCHES / 2;
        assertTrue("right, was " + dx, dx > 1);
        assertTrue("forward, was " + dy, dy > 1);
        assertTrue("more right than forward (0.5 against 0.28)", dx > dy);
    }

    // ------------------------------------------------------------------ choosing

    @Test
    public void aModeButtonDoesNothingUntilASideIsChosen() {
        press(X); // Launch - but no side has been chosen yet
        runner.start();
        cycle();

        assertEquals("Leave: forward at 0.5", 0.5, robot.frontLeft.power, 1e-9);
    }

    @Test
    public void theRightBumperGoesBackToChoosingASide() {
        press(A);  // Hive side
        press(RB); // changed my mind
        press(B);  // so B now means Wall side, not Park
        press(X);  // Launch, which on the Wall side starts by driving forward
        runner.start();
        cycle();

        assertEquals("full power forward", 1.0, robot.frontLeft.power, 1e-9);
        assertEquals(-1.0, robot.frontRight.power, 1e-9);
    }

    @Test
    public void aModeChoiceUsuallyNeverReachesTheDriverStation() {
        /*
         * init_loop adds "Park is Selected" on the press cycle only. A
         * telemetry.update() transmits only once 250 ms have passed since the
         * last transmission and clears the lines either way (TelemetryImpl,
         * SDK 11.0.0), so the line reaches the Driver Station only if the
         * press lands on that one cycle - about one cycle in twelve at 50 Hz.
         * This press does not, so the driver never sees which mode was chosen.
         */
        run(0.1);
        press(A);
        press(B);
        boolean seen = false;
        for (int i = 0; i < 100; i++) {
            cycle();
            seen |= robot.telemetry.visible().contains("Park is Selected");
        }
        assertFalse(seen);
    }

    // -------------------------------------------------------------------- launch

    @Test
    public void launchNeverRunsTheWheelsOrTheRamp() {
        /*
         * Almost certainly not what was meant, for two independent reasons.
         * Every LAUNCH task is built with target 0, and target is the power
         * passed to launcher.spin(), so the wheels are told 0. And the task
         * starts both its timers and, on the very next loop, takes the branch
         * that removes it - so the branch that would run the ramp is never
         * reached. It would throw if it were: ramp is never constructed.
         */
        press(A); // Hive side
        press(X); // Launch: a single LAUNCH task
        runner.start();

        double maxWheels = 0;
        double maxRamp = 0;
        for (int i = 0; i < 500; i++) {
            cycle();
            maxWheels = Math.max(maxWheels, Math.abs(robot.flywheelCommand()));
            maxRamp = Math.max(maxRamp, Math.abs(robot.rampFlow()));
        }
        assertEquals(0.0, maxWheels, 1e-9);
        assertEquals(0.0, maxRamp, 1e-9);
        assertNull("and nothing crashed", runner.failure());
    }

    @Test
    public void wallSideLaunchAndParkTurnsAtFivePercentPower() {
        /*
         * The ROTATE task's target is an angle in degrees: the code wraps it
         * into -180..180 and multiplies it by R = 24/360 to get a power. Launch
         * on the Wall side passes -90 (full power left). LaunchAndPark passes
         * -0.75, commented "Motor power", which comes out at -0.05 - a turn too
         * slow to notice in its one second.
         */
        press(B); // Wall side
        press(Y); // LaunchAndPark: forward 1.5 s, then the rotate
        runner.start();
        run(1.6);

        assertEquals(-0.75 * 2 * 12.0 / 360.0, robot.frontLeft.power, 1e-9);
        assertEquals(-0.05, robot.frontRight.power, 1e-9);
    }

    // ------------------------------------------------------------------- the end

    @Test
    public void theAutoStopsAtThirtySecondsWithEveryMotorStopped() {
        press(A);
        press(Y); // LaunchAndPark
        runner.start();
        run(31);

        assertEquals(OpModeRunner.Phase.STOPPED, runner.phase());
        assertEquals(OpModeRunner.AUTONOMOUS_SECONDS, runner.runSeconds(), DT);
        for (FakeHardware.Motor motor : robot.motors()) {
            assertEquals(motor.name, 0.0, motor.power, 1e-9);
        }
    }
}

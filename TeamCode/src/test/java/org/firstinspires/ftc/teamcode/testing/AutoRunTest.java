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
 * <p>The routes are BIOBUZZAutoOdo's waypoints, timed from the speeds the auto
 * is given. Those are the test UI's own speeds until the robot's are measured,
 * so here every route should end on its last waypoint.
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
        robot.updateOdometry(sim);
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
    private static final Consumer<Gamepad> LB = g -> g.left_bumper = true;

    /** Where the robot starts, so the Hive routes stay on the field. */
    private static final double HIVE_START_X = 90, HIVE_START_Y = 20;
    /** And the Wall routes, which go 100 in right. */
    private static final double WALL_START_X = 20, WALL_START_Y = 20;
    /** How close a timed route should end to its last waypoint here, inches. */
    private static final double ARRIVE_INCHES = 1.5;

    private double startX, startY;

    private void startAt(double x, double y) {
        startX = x;
        startY = y;
        sim.setStartingPose(x, y, 0);
        robot.resetOdometryOrigin(sim);
    }

    private double movedRight() { return sim.xInches - startX; }
    private double movedForward() { return sim.yInches - startY; }

    private void assertAt(double right, double forward, double clockwiseDeg) {
        assertEquals("right", right, movedRight(), ARRIVE_INCHES);
        assertEquals("forward", forward, movedForward(), ARRIVE_INCHES);
        assertEquals("heading", clockwiseDeg, sim.headingDeg, 2);
        assertNull(runner.failure());
    }

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
    public void drivetrainOnlyStillDrivesItsRoute() {
        unplug("intake");
        startAt(HIVE_START_X, HIVE_START_Y);
        press(A); // Hive side
        press(B); // Park
        runner.start();
        run(8);

        assertAt(-36, 76, 0);
    }

    // ------------------------------------------------------------------ routes

    @Test
    public void withNothingChosenItLaunchesFromTheHiveSideAndParks() {
        startAt(HIVE_START_X, HIVE_START_Y);
        runner.start();
        run(5.9);
        assertEquals("still launching", 0, Math.hypot(movedRight(), movedForward()), 0.01);
        assertTrue("flywheels on", robot.flywheelCommand() > 0);

        run(10);
        assertAt(-46, 96, 0);
    }

    @Test
    public void hiveParkEndsWhereTheOdometryAutoParks() {
        startAt(HIVE_START_X, HIVE_START_Y);
        press(A); // Hive side
        press(B); // Park
        runner.start();

        cycle();
        assertTrue("the intake runs while it drives", robot.intakeFlow() > 0);
        run(8);
        assertAt(-36, 76, 0);
        assertEquals("and stops after", 0, robot.intakeFlow(), 1e-9);
        assertEquals("stopped", 0, robot.frontLeft.power, 1e-9);
    }

    @Test
    public void wallParkGoesSixInchesForward() {
        startAt(WALL_START_X, WALL_START_Y);
        press(B); // Wall side
        press(B); // Park
        runner.start();
        run(3);

        assertAt(0, 6, 0);
    }

    @Test
    public void wallLaunchTurnsClockwiseToFaceRightThenLaunches() {
        startAt(WALL_START_X, WALL_START_Y);
        press(B); // Wall side
        press(X); // Launch
        runner.start();
        run(7.5); // there by about 6.5 s, then launching for 6

        // (0,12), (72,12), (72,36), turn to -90 (clockwise), (100,36) - the last
        // leg is "forward" for a robot that now faces right.
        assertAt(100, 36, 90);
        assertTrue("launching", robot.flywheelCommand() > 0);
    }

    @Test
    public void wallLaunchAndParkComesBackAcrossTheField() {
        startAt(WALL_START_X, WALL_START_Y);
        press(B); // Wall side
        press(Y); // LaunchAndPark
        runner.start();
        run(20);

        assertAt(0, 6, 90);
    }

    @Test
    public void hiveIntakeRouteEndsParked() {
        startAt(HIVE_START_X, HIVE_START_Y);
        press(A);  // Hive side
        press(LB); // Intake
        runner.start();
        run(29.9);

        assertAt(-46, 96, 0);
    }

    @Test
    public void wallIntakeRouteEndsParked() {
        startAt(WALL_START_X, WALL_START_Y);
        press(B);  // Wall side
        press(LB); // Intake
        runner.start();
        run(29.9);

        assertAt(0, 6, 90);
    }

    // ------------------------------------------------------------------ choosing

    @Test
    public void aModeButtonDoesNothingUntilASideIsChosen() {
        startAt(HIVE_START_X, HIVE_START_Y);
        press(X); // Launch - but no side has been chosen yet
        runner.start();
        run(9);

        // Still the default, LaunchAndPark: Launch alone would not have moved.
        assertTrue("drove off after launching, moved " + movedForward(), movedForward() > 5);
    }

    @Test
    public void theRightBumperGoesBackToChoosingASide() {
        press(A);  // Hive side
        press(RB); // changed my mind
        press(B);  // so B now means Wall side, not Park
        press(X);  // Launch, which on the Wall side starts by driving forward
        runner.start();
        cycle();

        assertEquals("forward at the drive power", 0.5, robot.frontLeft.power, 1e-9);
        assertEquals(-0.5, robot.frontRight.power, 1e-9);
        assertEquals(0.5, robot.rearLeft.power, 1e-9);
        assertEquals(-0.5, robot.rearRight.power, 1e-9);
    }

    @Test
    public void theChosenModeIsShownOnTheDriverStation() {
        run(0.1);
        press(A);
        press(B);
        boolean seen = false;
        for (int i = 0; i < 100; i++) {
            cycle();
            seen |= robot.telemetry.visible().contains("Park is Selected");
        }
        assertTrue(seen);
    }

    // -------------------------------------------------------------------- launch

    @Test
    public void aLaunchSpinsUpThenFeedsThenStops() {
        press(A); // Hive side
        press(X); // Launch: a single LAUNCH task
        runner.start();

        run(2.9);
        assertEquals("spinning up", 0.85, robot.flywheelCommand(), 1e-9);
        assertEquals("not feeding yet", 0, robot.rampFlow(), 1e-9);
        run(1.0);
        assertTrue("feeding", Math.abs(robot.rampFlow()) > 0);
        run(2.5);
        assertEquals("done", 0, robot.flywheelCommand(), 1e-9);
        assertEquals(0, robot.rampFlow(), 1e-9);
        assertNull(runner.failure());
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

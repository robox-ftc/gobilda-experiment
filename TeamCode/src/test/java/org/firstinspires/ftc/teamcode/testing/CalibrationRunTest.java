package org.firstinspires.ftc.teamcode.testing;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.calibration.CalDriveMotors;
import org.firstinspires.ftc.teamcode.calibration.CalDriveSpeed;
import org.firstinspires.ftc.teamcode.calibration.CalPinpoint;
import org.firstinspires.ftc.teamcode.devices.PinpointSetup;
import org.junit.After;
import org.junit.Test;

import java.util.function.Consumer;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * The calibration OpModes, run on the simulated robot: what they report for a
 * robot that matches the test UI, and that they catch the faults they are for.
 * What they report here is also what the test UI shows, so it is the baseline
 * to compare the real robot's results with.
 */
public class CalibrationRunTest {

    private static final double DT = 0.02;

    private final DriverStation ds = new DriverStation();
    private final RobotSim sim = new RobotSim();
    private DesktopRobot robot;
    private OpModeRunner runner;

    @After
    public void closeRunner() {
        if (runner != null) runner.close();
    }

    private void setUp(Class<? extends OpMode> opMode, SimPinpoint.Mounting mounting) {
        if (runner != null) runner.close();
        sim.setStartingPose(72, 72, 0); // the middle of the field: room every way
        robot = new DesktopRobot();
        robot.odometry.mounting = mounting;
        robot.resetOdometryOrigin(sim);
        runner = new OpModeRunner(opMode, robot, ds);
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

    private void press(Consumer<Gamepad> button) {
        Gamepad held = new Gamepad();
        button.accept(held);
        ds.held1.copy(held);
        cycle();
        ds.releaseAll();
        cycle();
    }

    private String screen() {
        return String.join("\n", robot.telemetry.visible());
    }

    /** The line on screen starting with {@code prefix}. */
    private String line(String prefix) {
        for (String l : robot.telemetry.visible()) if (l.startsWith(prefix)) return l;
        throw new AssertionError("no line starting '" + prefix + "' in:\n" + screen());
    }

    private static double number(String text, String regex) {
        Matcher m = Pattern.compile(regex).matcher(text);
        assertTrue("'" + regex + "' not in: " + text, m.find());
        return Double.parseDouble(m.group(1));
    }

    // ------------------------------------------------------------ Cal 1: motors

    private void runDriveMotors() {
        run(0.5);
        runner.start();
        run(16);
        assertNull(runner.failure());
    }

    @Test
    public void driveMotorsAllPassOnAHealthyRobot() {
        setUp(CalDriveMotors.class, new SimPinpoint.Mounting());
        runDriveMotors();
        for (String name : new String[]{"lf", "rf", "lb", "rb"}) {
            assertTrue(screen(), line(name + ":").startsWith(name + ": OK"));
        }
    }

    @Test
    public void anUnpluggedEncoderIsFound() {
        setUp(CalDriveMotors.class, new SimPinpoint.Mounting());
        robot.rearLeft.encoder = null;
        runDriveMotors();
        assertTrue(screen(), line("lb:").contains("NO ENCODER COUNTS"));
        assertTrue(screen(), line("lf:").startsWith("lf: OK"));
        assertTrue(screen(), line("rb:").startsWith("rb: OK"));
    }

    @Test
    public void crossedEncoderCablesAreFound() {
        setUp(CalDriveMotors.class, new SimPinpoint.Mounting());
        robot.frontLeft.encoder = robot.frontRight;
        robot.frontRight.encoder = robot.frontLeft;
        runDriveMotors();
        assertTrue(screen(), line("lf:").contains("WRONG PORT"));
        assertTrue(screen(), line("lf:").contains("rf"));
        assertTrue(screen(), line("rf:").contains("WRONG PORT"));
        assertTrue(screen(), line("lb:").startsWith("lb: OK"));
    }

    // ---------------------------------------------------------- Cal 2: Pinpoint

    /** Moves the simulated robot as a person pushing it would, a little at a time. */
    private void push(double rightInches, double forwardInches, double cwDegrees) {
        int steps = 40;
        for (int i = 0; i < steps; i++) {
            sim.xInches += rightInches / steps;
            sim.yInches += forwardInches / steps;
            sim.headingDeg += cwDegrees / steps;
            cycle();
        }
    }

    private void doTheThreeChecks() {
        run(0.5); // gyro calibrates
        push(0, 24, 0);
        press(g -> g.a = true);
        push(-24, 0, 0);
        press(g -> g.a = true);
        push(0, 0, -90); // counterclockwise
        press(g -> g.a = true);
        run(0.3);
    }

    @Test
    public void aPinpointFittedAsTheCodeSaysPasses() {
        setUp(CalPinpoint.class, new SimPinpoint.Mounting());
        doTheThreeChecks();
        assertTrue(screen(), screen().contains("DIRECTIONS: all correct"));

        double before = sim.headingDeg;
        runner.start();
        run(12);
        assertNull(runner.failure());
        assertTrue(screen(), screen().contains("OFFSETS: within half an inch"));
        assertEquals("turned back to where it started", before, sim.headingDeg, 5);
    }

    @Test
    public void aBackwardsXPodIsCaughtAndTheFixNamed() {
        SimPinpoint.Mounting xReversed = new SimPinpoint.Mounting();
        xReversed.forwardPodSign = -xReversed.forwardPodSign;
        setUp(CalPinpoint.class, xReversed);
        doTheThreeChecks();
        assertTrue(screen(), line("Forward:").endsWith("WRONG."));
        assertTrue(screen(), line("Left:").endsWith("OK."));
        // The forward pod is in the Y port when mirrored, the X port otherwise.
        String port = PinpointSetup.MIRRORED ? "Y" : "X";
        String other = PinpointSetup.MIRRORED ? "X" : "Y";
        assertTrue(screen(), screen().contains(port + "_POD_DIRECTION = REVERSED"));
        assertFalse(screen(), screen().contains(other + "_POD_DIRECTION"));
        assertFalse(screen(), screen().contains("MIRRORED ="));

        double before = sim.headingDeg;
        runner.start();
        run(2);
        assertTrue("does not spin with a bad pod", screen().contains("Offset check skipped"));
        assertEquals(before, sim.headingDeg, 1e-9);
    }

    @Test
    public void aPinpointMirroredDifferentlyFromTheCodeIsCaughtAndTheFixNamed() {
        // As on 2026-11-20, the other way round: the code and the robot
        // disagree about MIRRORED, so forward reads sideways.
        SimPinpoint.Mounting other = new SimPinpoint.Mounting();
        other.mirrored = !PinpointSetup.MIRRORED;
        setUp(CalPinpoint.class, other);
        doTheThreeChecks();
        assertTrue(screen(), line("Forward:").endsWith("WRONG."));
        assertTrue(screen(), line("Left:").endsWith("WRONG."));
        assertTrue(screen(), line("Turn:").endsWith("WRONG."));
        assertTrue(screen(), screen().contains("MIRRORED = " + !PinpointSetup.MIRRORED));

        runner.start();
        run(2);
        assertTrue("does not spin with the directions wrong", screen().contains("Offset check skipped"));
    }

    @Test
    public void wrongOffsetsAreMeasuredByTurning() {
        SimPinpoint.Mounting moved = new SimPinpoint.Mounting();
        moved.forwardPodLeftInches = 3.0;
        moved.strafePodForwardInches = 5.0;
        setUp(CalPinpoint.class, moved);
        doTheThreeChecks();
        runner.start();
        run(12);
        assertNull(runner.failure());

        String found = line("OFFSETS look like");
        assertEquals(found, 3.0, number(found, "FORWARD_POD_LEFT_INCHES = (-?[\\d.]+)"), 0.3);
        assertEquals(found, 5.0, number(found, "STRAFE_POD_FORWARD_INCHES = (-?[\\d.]+)"), 0.3);
        assertTrue(screen(), screen().contains("--pinpoint=xpod=3.0,ypod=5.0"));
    }

    @Test
    public void turningAgainstAWallIsReportedNotTakenForWrongOffsets() {
        // The test UI's default start: backed onto the audience wall. Turning
        // there pushes the robot 3.5 in off the wall (its corners stick out),
        // which on 2026-11-18 came out as "Y_POD_FORWARD_INCHES = 6.6".
        setUp(CalPinpoint.class, new SimPinpoint.Mounting());
        sim.setStartingPose(72, RobotSim.ROBOT_INCHES / 2, 0);
        robot.resetOdometryOrigin(sim);
        run(0.5);
        runner.start(); // without the INIT checks, as in that run
        run(12);
        assertNull(runner.failure());

        assertTrue(screen(), screen().contains("OFFSETS: NOT MEASURED"));
        assertFalse(screen(), screen().contains("OFFSETS look like"));
        assertFalse(screen(), screen().contains("--pinpoint="));
        assertTrue(screen(), screen().contains("direction checks in INIT were not all done"));
    }

    @Test
    public void aShiftClickInTheCornerStillLeavesRoomToTurn() {
        setUp(CalPinpoint.class, new SimPinpoint.Mounting());
        GamepadHarness.placeFreely(sim, new java.awt.geom.Point2D.Double(0, 0));
        robot.resetOdometryOrigin(sim);
        assertTrue("off the wall, was " + sim.xInches, sim.xInches > RobotSim.ROBOT_INCHES / 2 * Math.sqrt(2));
        assertEquals(sim.xInches, sim.yInches, 1e-9);

        run(0.5);
        runner.start();
        run(12);
        assertNull(runner.failure());
        assertTrue(screen(), screen().contains("OFFSETS: within half an inch"));
    }

    // ------------------------------------------------------- Cal 3: drive speed

    @Test
    public void driveSpeedReportsWhatTheTestUiAssumes() {
        setUp(CalDriveSpeed.class, new SimPinpoint.Mounting());
        run(0.5);
        runner.start();
        run(23);
        assertNull(runner.failure());

        String forward = line("Forward at full power");
        assertEquals(forward, RobotSim.MAX_SPEED_IPS, number(forward, "power: ([\\d.]+) in/s"), 1.0);
        assertEquals(forward, 2.0, number(forward, "ran ([\\d.]+)x"), 0.1);
        String sideways = line("Sideways at full power");
        assertEquals(sideways, RobotSim.MAX_SPEED_IPS * RobotSim.STRAFE_FRACTION,
                number(sideways, "power: ([\\d.]+) in/s"), 1.0);
        String turning = line("Turning at full power");
        assertEquals(turning, RobotSim.MAX_TURN_DPS, number(turning, "power: ([\\d.]+) deg/s"), 5.0);
        assertFalse(screen(), screen().contains("DOES NOT FOLLOW POWER"));

        assertEquals("ends near the start, x", 72, sim.xInches, 3);
        assertEquals("ends near the start, y", 72, sim.yInches, 3);
        assertFalse(screen(), screen().contains("BLOCKED"));
    }

    @Test
    public void aMoveStoppedByAWallIsLeftOutOfTheSpeeds() {
        // 25 in from the right wall: the 0.6 move right (34 in) hits it, as the
        // 2026-11-18 test UI run hit the FLOWER and reported sideways at 91%.
        setUp(CalDriveSpeed.class, new SimPinpoint.Mounting());
        sim.setStartingPose(RobotSim.FIELD_INCHES - RobotSim.ROBOT_INCHES / 2 - 25, 72, 0);
        robot.resetOdometryOrigin(sim);
        run(0.5);
        runner.start();
        run(23);
        assertNull(runner.failure());

        assertTrue(screen(), line("BLOCKED").startsWith("BLOCKED: right 0.6"));
        String sideways = line("Sideways at full power");
        assertEquals(sideways, RobotSim.MAX_SPEED_IPS, number(sideways, "power: ([\\d.]+) in/s"), 1.0);
        assertTrue(sideways, sideways.contains("from one power only"));
        String forward = line("Forward at full power");
        assertTrue("forward was not blocked: " + forward, forward.contains("ran 2.0x"));
    }
}

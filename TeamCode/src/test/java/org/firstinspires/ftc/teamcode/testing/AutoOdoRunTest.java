package org.firstinspires.ftc.teamcode.testing;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.BIOBUZZAutoOdo;
import org.firstinspires.ftc.teamcode.devices.PinpointSetup;
import org.junit.After;
import org.junit.Test;

import java.util.function.Consumer;

/**
 * HiveKnightAuto-Odometry ({@link BIOBUZZAutoOdo}), the real class, on stand-in
 * motors and a {@link SimPinpoint} that can be fitted wrong the way the real
 * robot's was on 2026-11-18.
 */
public class AutoOdoRunTest {

    private static final double DT = 0.02;

    // Hive-side Park: (-12, 12), then (-36, 76) - inches right and forward of the start.
    private static final double PARK_RIGHT = -36;
    private static final double PARK_FORWARD = 76;

    // Far enough from the walls for the whole Park.
    private static final double START_X = 90;
    private static final double START_Y = 20;

    private final DriverStation ds = new DriverStation();
    private final RobotSim sim = new RobotSim();
    private DesktopRobot robot;
    private OpModeRunner runner;

    @After
    public void closeRunner() {
        if (runner != null) runner.close();
    }

    /** The robot put down at the start, then the OpMode INITed, as on the field. */
    private void setUp(SimPinpoint.Mounting mounting, String... unplugged) {
        if (runner != null) runner.close();
        sim.setStartingPose(START_X, START_Y, 0);
        robot = new DesktopRobot(unplugged);
        robot.odometry.mounting = mounting;
        robot.resetOdometryOrigin(sim);
        runner = new OpModeRunner(BIOBUZZAutoOdo.class, robot, ds);
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

    private void chooseHiveSidePark() {
        run(0.5); // the gyro calibrates
        press(g -> g.a = true); // Hive side
        press(g -> g.b = true); // Park
    }

    private double movedRight() { return sim.xInches - START_X; }
    private double movedForward() { return sim.yInches - START_Y; }
    private String screen() { return String.join("\n", robot.telemetry.visible()); }

    private double pinpointHeading() {
        return PinpointSetup.headingDegrees(robot.pinpoint);
    }

    @Test
    public void fittedAsTheCodeSaysItParksWhereItWasSent() {
        setUp(new SimPinpoint.Mounting());
        chooseHiveSidePark();
        runner.start();
        run(8);

        assertNull(runner.failure());
        assertEquals("right", PARK_RIGHT, movedRight(), 3.5);
        assertEquals("forward", PARK_FORWARD, movedForward(), 3.5);
        assertEquals("still facing forward", 0, sim.headingDeg, 2);
        assertEquals("stopped", 0, robot.frontLeft.power, 1e-9);
    }

    @Test
    public void withTheXPodBackwardsItStopsItselfInsteadOfRunningAway() {
        /*
         * The 2026-11-18 run: every step took it further from the target
         * until someone pressed STOP. Now the auto sees the distance growing
         * and stops within a fraction of a second.
         */
        SimPinpoint.Mounting xReversed = new SimPinpoint.Mounting();
        xReversed.forwardPodSign = -xReversed.forwardPodSign;
        setUp(xReversed);
        chooseHiveSidePark();
        runner.start();
        run(3);

        assertNull(runner.failure());
        assertTrue(screen(), screen().contains("ran away"));
        for (FakeHardware.Motor motor : robot.motors()) {
            assertEquals(motor.name, 0.0, motor.power, 1e-9);
        }
        double travelled = Math.hypot(movedRight(), movedForward());
        assertTrue("stopped early, travelled " + travelled, travelled < 25); // about 22 here, 7 of it braking from full speed
    }

    @Test
    public void aKnockMidMoveIsTurnedBackOut() {
        setUp(new SimPinpoint.Mounting());
        chooseHiveSidePark();
        runner.start();
        run(1.0);

        sim.headingDeg += 25; // bumped clockwise
        robot.updateOdometry(sim);
        run(0.6);
        assertTrue("turning back, was " + sim.headingDeg, Math.abs(sim.headingDeg) < 10);

        run(7);
        assertEquals("facing forward again", 0, sim.headingDeg, 2);
        assertEquals("right", PARK_RIGHT, movedRight(), 3.5);
        assertEquals("forward", PARK_FORWARD, movedForward(), 3.5);
    }

    @Test
    public void aDriftingGyroIsReportedInInitAndForgottenAtStart() {
        SimPinpoint.Mounting drifting = new SimPinpoint.Mounting();
        drifting.gyroDriftDps = 4;
        setUp(drifting);
        chooseHiveSidePark();
        run(14);

        assertTrue("heading has wandered, was " + pinpointHeading(), pinpointHeading() > 45);
        assertTrue(screen(), screen().contains("GYRO DRIFTING"));

        runner.start();
        robot.pinpoint.update();
        assertEquals("zeroed at START", 0, pinpointHeading(), 0.5);
        assertEquals(0, robot.pinpoint.getPosX(DistanceUnit.INCH), 0.5);
    }

    @Test
    public void dpadDownInInitRecalibratesTheGyro() {
        // Picked up and turned while the gyro calibrated, so it drifts after.
        setUp(new SimPinpoint.Mounting());
        robot.pinpoint.resetPosAndIMU();
        for (int i = 0; i < 15; i++) {
            sim.headingDeg += 1;
            cycle();
        }
        run(3);
        assertTrue(screen(), screen().contains("GYRO DRIFTING"));

        press(g -> g.dpad_down = true);
        run(0.5);
        double before = pinpointHeading();
        run(3);
        assertEquals("steady now", before, pinpointHeading(), 0.1);
        assertTrue(screen(), !screen().contains("GYRO DRIFTING"));
    }

    @Test
    public void withNoPinpointItSaysSoAndNeverMoves() {
        setUp(new SimPinpoint.Mounting(), "pinpoint");
        run(0.5);
        assertTrue(screen(), screen().contains("NO PINPOINT"));

        runner.start();
        run(2);
        assertNull(runner.failure());
        assertEquals(0, robot.frontLeft.power, 1e-9);
        assertEquals(START_Y, sim.yInches, 1e-9);
    }
}

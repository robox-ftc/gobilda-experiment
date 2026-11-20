package org.firstinspires.ftc.teamcode.testing;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.devices.PinpointSetup;
import org.junit.Test;

/**
 * The simulated Pinpoint has to get things wrong the way the real one does
 * when it is set up wrong - otherwise the harness passes code the robot fails.
 */
public class SimPinpointTest {

    private static final double DT = 0.02;

    private final SimClock clock = new SimClock();
    private final SimPinpoint odometry = new SimPinpoint(clock::nanos);
    private final GoBildaPinpointDriver pinpoint = odometry.device;
    private final RobotSim sim = new RobotSim();

    /** As the robot code configures it (PinpointSetup), then zeroed where the robot stands. */
    private void configureAsTheCodeDoes() {
        PinpointSetup.configure(pinpoint);
        odometry.placeAt(sim);
    }

    private static EncoderDirection flip(EncoderDirection d) {
        return d == EncoderDirection.FORWARD ? EncoderDirection.REVERSED : EncoderDirection.FORWARD;
    }

    private void drive(double fl, double fr, double rl, double rr, double seconds) {
        long n = Math.round(seconds / DT);
        for (long i = 0; i < n; i++) {
            sim.step(fl, fr, rl, rr, DT);
            clock.advance(DT);
            odometry.track(sim);
        }
        pinpoint.update();
    }

    private void forward(double seconds) { drive(1, -1, 1, -1, seconds); }
    private void left(double seconds) { drive(-1, -1, 1, 1, seconds); }
    private void counterclockwise(double seconds) { drive(-0.5, -0.5, -0.5, -0.5, seconds); }
    private void still(double seconds) { drive(0, 0, 0, 0, seconds); }

    // Forward, left and counterclockwise, as the robot code reads them.
    private double x() { return PinpointSetup.forwardInches(pinpoint); }
    private double y() { return PinpointSetup.leftInches(pinpoint); }
    private double heading() { return PinpointSetup.headingDegrees(pinpoint); }

    @Test
    public void fittedAsTheCodeAssumesItReadsForwardAndLeftPositive() {
        configureAsTheCodeDoes();
        double startY = sim.yInches;

        forward(0.5);
        assertEquals(sim.yInches - startY, x(), 0.01);
        assertEquals(0, y(), 0.01);
        assertEquals(0, heading(), 0.01);

        // Still rolling forward as it starts to strafe, so check both axes.
        still(1.0);
        double xBefore = x();
        double startX = sim.xInches;
        double startY2 = sim.yInches;
        left(0.3);
        assertEquals(sim.yInches - startY2 + xBefore, x(), 0.01);
        assertTrue("moved left, was " + y(), y() > 5);
        assertEquals(startX - sim.xInches, y(), 0.01);
    }

    @Test
    public void itsFrameIsTheWayTheRobotFacedWhenZeroedNotTheField() {
        sim.setStartingPose(72, 72, 90); // facing the right-hand wall
        configureAsTheCodeDoes();

        forward(0.3);
        assertTrue("forward is +X, was " + x(), x() > 5);
        assertEquals(0, y(), 0.01);
    }

    @Test
    public void aReversedXPodReadsDrivingForwardAsBackward() {
        odometry.mounting.forwardPodSign = -odometry.mounting.forwardPodSign;
        configureAsTheCodeDoes();

        forward(0.5);
        assertTrue("backward, was " + x(), x() < -5);
    }

    @Test
    public void configuringTheXPodReversedUndoesAReversedMounting() {
        odometry.mounting.forwardPodSign = -odometry.mounting.forwardPodSign;
        configureAsTheCodeDoes();
        // Flip whichever port the forward pod is in: Y when mirrored.
        pinpoint.setEncoderDirections(
                PinpointSetup.MIRRORED ? PinpointSetup.X_POD_DIRECTION : flip(PinpointSetup.X_POD_DIRECTION),
                PinpointSetup.MIRRORED ? flip(PinpointSetup.Y_POD_DIRECTION) : PinpointSetup.Y_POD_DIRECTION);

        forward(0.5);
        assertTrue("forward, was " + x(), x() > 5);
    }

    @Test
    public void theGettersHoldWhatTheLastUpdateRead() {
        configureAsTheCodeDoes();
        forward(0.5);
        double atUpdate = x();

        sim.step(1, -1, 1, -1, DT);
        clock.advance(DT);
        odometry.track(sim);
        assertEquals("no update() yet", atUpdate, x(), 1e-12);

        pinpoint.update();
        assertTrue(x() > atUpdate);
    }

    @Test
    public void aDriftingGyroLooksLikeTheRobotSlidingSideways() {
        /*
         * The 2026-11-18 run, in INIT with the robot still: heading 60.49 ->
         * 62.43 in half a second while X/Y moved (0.22, -0.12). That is the
         * device taking the strafe pod's 7.5 in arc back out for a turn the
         * pods never felt.
         */
        odometry.mounting.gyroDriftDps = 3.9;
        configureAsTheCodeDoes();

        still(0.5);
        assertEquals(1.95, heading(), 0.01);
        double expectedSlide = Math.toRadians(1.95) * 7.5;
        assertEquals(expectedSlide, Math.hypot(x(), y()), 0.01);
        assertTrue("to the right, was " + y(), y() < 0);
        assertEquals(0, sim.xInches - 72, 1e-9);
    }

    @Test
    public void withTheRightOffsetsTurningInPlaceDoesNotMoveIt() {
        configureAsTheCodeDoes();

        counterclockwise(0.5);
        assertTrue("turned, was " + heading(), heading() > 30);
        assertEquals(0, x(), 0.01);
        assertEquals(0, y(), 0.01);
    }

    @Test
    public void withTheWrongOffsetsTurningInPlaceLooksLikeSliding() {
        odometry.mounting.strafePodForwardInches = -4; // really behind the centre
        configureAsTheCodeDoes();

        counterclockwise(0.5);
        assertTrue("slid, was " + Math.hypot(x(), y()), Math.hypot(x(), y()) > 3);
    }

    @Test
    public void movingWhileItCalibratesLeavesTheGyroDrifting() {
        configureAsTheCodeDoes();

        pinpoint.resetPosAndIMU(); // ...while the robot is being turned
        assertEquals(GoBildaPinpointDriver.DeviceStatus.CALIBRATING, pinpoint.getDeviceStatus());
        counterclockwise(SimPinpoint.CALIBRATION_SECONDS);
        assertEquals(GoBildaPinpointDriver.DeviceStatus.READY, pinpoint.getDeviceStatus());

        // It took the counterclockwise turn as its zero, so a robot that then
        // stops looks as if it is turning clockwise.
        still(0.5); // let the robot come to rest
        double before = heading();
        still(1.0);
        double drift = heading() - before;
        assertTrue("still, but drifting clockwise, was " + drift, drift < -30);
    }

    @Test
    public void theRealRobotsPinpointReadsAsCal2FoundIt() {
        /*
         * Cal 2 on 2026-11-20, with the code not yet mirroring: pushed forward,
         * the Pinpoint's Y went down (-39.9); pushed left, its X went down
         * (-48.7); turned counterclockwise, its heading went down (-78.4).
         */
        odometry.mounting.mirrored = true;
        pinpoint.setEncoderDirections(EncoderDirection.FORWARD, EncoderDirection.FORWARD);
        pinpoint.setOffsets(0, 0, DistanceUnit.INCH);
        odometry.mounting.forwardPodSign = -1; // counts the robot moving backward
        odometry.mounting.strafePodSign = -1;  // counts it moving right
        odometry.mounting.forwardPodLeftInches = 0;
        odometry.mounting.strafePodForwardInches = 0;
        odometry.placeAt(sim);

        forward(0.4);
        assertTrue("raw Y down, was " + pinpoint.getPosY(DistanceUnit.INCH), pinpoint.getPosY(DistanceUnit.INCH) < -5);
        assertEquals(0, pinpoint.getPosX(DistanceUnit.INCH), 0.01);
        still(1.0);
        odometry.placeAt(sim);
        left(0.4);
        assertTrue("raw X down, was " + pinpoint.getPosX(DistanceUnit.INCH), pinpoint.getPosX(DistanceUnit.INCH) < -5);
        still(1.0);
        odometry.placeAt(sim);
        counterclockwise(0.4);
        assertTrue("raw heading down, was " + pinpoint.getHeading(AngleUnit.DEGREES),
                pinpoint.getHeading(AngleUnit.DEGREES) < -10);
    }

    @Test
    public void mirroredReadingsTurnBackIntoTheRobotsMotionEvenAfterTurning() {
        // The point of reading a mirrored Pinpoint through PinpointSetup: it
        // must still be right once the robot has turned, not just at heading 0.
        odometry.mounting.mirrored = PinpointSetup.MIRRORED;
        configureAsTheCodeDoes();
        double startX = sim.xInches, startY = sim.yInches;

        counterclockwise(0.4);
        still(1.0);
        double turned = heading();
        assertEquals(-sim.headingDeg, turned, 0.5);
        forward(0.4);
        still(1.0);
        // Field movement, back into the frame the robot faced when zeroed.
        double dx = sim.xInches - startX, dy = sim.yInches - startY;
        assertEquals("forward", dy, x(), 0.1);
        assertEquals("left", -dx, y(), 0.1);
    }

    @Test
    public void mirroredOffsetsTakeTurningBackOutWhereverThePodsAre() {
        // Both pods off-centre, so a sign slip in either converted offset shows.
        odometry.mounting.mirrored = true;
        odometry.mounting.forwardPodSign = -1;
        odometry.mounting.strafePodSign = -1;
        odometry.mounting.forwardPodLeftInches = 3.0;
        odometry.mounting.strafePodForwardInches = -4.5;
        pinpoint.setEncoderDirections(EncoderDirection.FORWARD, EncoderDirection.FORWARD);
        double[] offsets = PinpointSetup.deviceOffsets(true, 3.0, -4.5);
        pinpoint.setOffsets(offsets[0], offsets[1], DistanceUnit.INCH);
        odometry.placeAt(sim);

        counterclockwise(0.5);
        assertTrue("turned, was " + pinpoint.getHeading(AngleUnit.DEGREES),
                Math.abs(pinpoint.getHeading(AngleUnit.DEGREES)) > 30);
        assertEquals(0, pinpoint.getPosX(DistanceUnit.INCH), 0.01);
        assertEquals(0, pinpoint.getPosY(DistanceUnit.INCH), 0.01);
    }

    @Test
    public void mountingParsesTheHarnessOption() {
        SimPinpoint.Mounting fitted = new SimPinpoint.Mounting();
        SimPinpoint.Mounting m = SimPinpoint.Mounting.parse("x-reversed, drift=4,ypod=-3");
        assertEquals("reversed from what the code expects", -fitted.forwardPodSign, m.forwardPodSign);
        assertEquals(fitted.strafePodSign, m.strafePodSign);
        assertEquals(PinpointSetup.MIRRORED, m.mirrored);
        assertEquals(!PinpointSetup.MIRRORED,
                SimPinpoint.Mounting.parse(PinpointSetup.MIRRORED ? "not-mirrored" : "mirrored").mirrored);
        assertEquals(4, m.gyroDriftDps, 0);
        assertEquals(-3, m.strafePodForwardInches, 0);
    }
}

package org.firstinspires.ftc.teamcode.testing;

import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.anyDouble;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.devices.PinpointSetup;
import org.mockito.Mockito;

import java.util.function.LongSupplier;

/**
 * The goBILDA Pinpoint, modelled from its two pods and its gyro rather than
 * copied from the field simulator.
 *
 * <p>The earlier stand-in handed the OpMode the simulator's true pose. That
 * could never be wrong, so it hid every Pinpoint setup mistake - and on
 * 2026-11-18 HiveKnightAuto-Odometry drove away from its target on the real
 * robot while it parked perfectly here. This one works the way the device
 * does:
 *
 * <ul>
 *   <li>Each pod measures how far the floor moved under <em>it</em>, along its
 *       own axis, counted the way it happens to be {@link Mounting mounted}.
 *       The OpMode's {@code setEncoderDirections} must undo that mounting, or
 *       the readings come out mirrored.</li>
 *   <li>Heading comes from the gyro, not the pods. It drifts at
 *       {@link Mounting#gyroDriftDps}, plus whatever the robot was turning at
 *       while the gyro calibrated ({@code resetPosAndIMU}/{@code
 *       recalibrateIMU} take {@value #CALIBRATION_SECONDS} s and assume the
 *       robot is still).</li>
 *   <li>The pods sit off-centre, so turning moves them. The device takes that
 *       back out using the OpMode's {@code setOffsets}; if those are not where
 *       the pods really are, turning shows up as sliding.</li>
 *   <li>The getters return what the last {@code update()} read, as on the
 *       robot.</li>
 * </ul>
 *
 *   <li>It can be fitted as a mirror image ({@link Mounting#mirrored}), as
 *       the real one is: pods in each other's ports and the board upside
 *       down, so its X counts the robot moving right, its Y moving backward,
 *       and its heading clockwise. {@code PinpointSetup.MIRRORED} undoes that.</li>
 * </ul>
 *
 * <p>The getters are in the Pinpoint's own frame; read them through
 * {@code PinpointSetup} for forward, left and counterclockwise, as the
 * robot code does.
 */
public final class SimPinpoint {

    /** How long the device spends recalibrating its gyro (goBILDA: about 0.25 s). */
    public static final double CALIBRATION_SECONDS = 0.25;

    /**
     * How the Pinpoint and its pods are really fitted to the robot - what a
     * bench test finds, not what the code says. A new one is fitted the way
     * {@link PinpointSetup} says, which Cal 2 found on the real robot; change
     * it (or pass {@code --pinpoint=} to the harness) to try a robot that
     * differs from the code.
     */
    public static final class Mounting {
        /**
         * Pods in each other's ports and the board upside down, as on the real
         * robot: the X port has the sideways pod and the Y port the forward
         * one, and the gyro reads clockwise as positive.
         */
        public boolean mirrored = PinpointSetup.MIRRORED;
        /** +1 if the forward-measuring pod counts up as the robot drives forward, -1 if it counts down. */
        public int forwardPodSign = codeForwardPodSign();
        /** +1 if the sideways-measuring pod counts up as the robot moves left, -1 if it counts down. */
        public int strafePodSign = codeStrafePodSign();
        /**
         * Where the forward-measuring pod really is: inches left of the
         * robot's centre. What PinpointSetup says; nobody has measured it.
         */
        public double forwardPodLeftInches = PinpointSetup.FORWARD_POD_LEFT_INCHES;
        /** Where the sideways pod really is: inches forward of the robot's centre. */
        public double strafePodForwardInches = PinpointSetup.STRAFE_POD_FORWARD_INCHES;
        /**
         * Gyro drift after a clean calibration, degrees per second,
         * counterclockwise-positive as the robot code reads it (through
         * PinpointSetup), whichever way up the board is.
         */
        public double gyroDriftDps = 0;

        /** How the forward pod must count for the code's settings to be right. */
        private static int codeForwardPodSign() {
            // Mirrored, it is in the Y port, which must count the robot moving backward.
            return PinpointSetup.MIRRORED ? -sign(PinpointSetup.Y_POD_DIRECTION) : sign(PinpointSetup.X_POD_DIRECTION);
        }

        /** How the sideways pod must count for the code's settings to be right. */
        private static int codeStrafePodSign() {
            // Mirrored, it is in the X port, which must count the robot moving right.
            return PinpointSetup.MIRRORED ? -sign(PinpointSetup.X_POD_DIRECTION) : sign(PinpointSetup.Y_POD_DIRECTION);
        }

        private static int sign(GoBildaPinpointDriver.EncoderDirection d) {
            return d == GoBildaPinpointDriver.EncoderDirection.REVERSED ? -1 : 1;
        }

        public Mounting copy() {
            Mounting m = new Mounting();
            m.mirrored = mirrored;
            m.forwardPodSign = forwardPodSign;
            m.strafePodSign = strafePodSign;
            m.forwardPodLeftInches = forwardPodLeftInches;
            m.strafePodForwardInches = strafePodForwardInches;
            m.gyroDriftDps = gyroDriftDps;
            return m;
        }

        /**
         * Reads a comma-separated description, as given to the harness, of
         * how the real Pinpoint differs from what the code says:
         * {@code x-reversed} (the forward pod counts the other way),
         * {@code y-reversed} (the sideways pod does), {@code mirrored} or
         * {@code not-mirrored}, {@code drift=<deg/s>},
         * {@code xpod=<in left>}, {@code ypod=<in forward>}. Anything not
         * mentioned keeps its default.
         */
        public static Mounting parse(String spec) {
            Mounting m = new Mounting();
            for (String raw : spec.split(",")) {
                String part = raw.trim().toLowerCase();
                if (part.isEmpty()) continue;
                if (part.equals("x-reversed")) m.forwardPodSign = -m.forwardPodSign;
                else if (part.equals("y-reversed")) m.strafePodSign = -m.strafePodSign;
                else if (part.equals("mirrored")) m.mirrored = true;
                else if (part.equals("not-mirrored")) m.mirrored = false;
                else if (part.startsWith("drift=")) m.gyroDriftDps = number(part);
                else if (part.startsWith("xpod=")) m.forwardPodLeftInches = number(part);
                else if (part.startsWith("ypod=")) m.strafePodForwardInches = number(part);
                else throw new IllegalArgumentException("unknown Pinpoint mounting '" + raw
                            + "': use x-reversed, y-reversed, mirrored, not-mirrored, drift=, xpod=, ypod=");
            }
            return m;
        }

        private static double number(String part) {
            return Double.parseDouble(part.substring(part.indexOf('=') + 1));
        }

        @Override
        public String toString() {
            return String.format("%sforward pod counting %s at %.1f in left, sideways pod counting %s at %.1f in"
                            + " forward, gyro drift %.1f deg/s",
                    mirrored ? "MIRRORED (pods in each other's ports, board upside down), " : "",
                    forwardPodSign > 0 ? "forward" : "backward", forwardPodLeftInches,
                    strafePodSign > 0 ? "left" : "right", strafePodForwardInches, gyroDriftDps);
        }
    }

    /** What the OpMode is given from the hardware map. */
    public final GoBildaPinpointDriver device = Mockito.mock(GoBildaPinpointDriver.class);

    /** The real robot. Set before the OpMode runs. */
    public Mounting mounting = new Mounting();

    private final LongSupplier clockNanos;

    // How the OpMode configured the device. The device's defaults: both
    // encoders FORWARD and both offsets 0.
    private int xDirection = 1;
    private int yDirection = 1;
    private double xOffsetInches;
    private double yOffsetInches;

    // The device's running estimate, which it keeps up to date by itself.
    private double estX;
    private double estY;
    private double estHeadingRad;

    // What the getters return: the estimate as of the last update().
    private double readX;
    private double readY;
    private double readHeadingRad;

    // Gyro calibration.
    private long calibrationEndsNanos;
    private boolean calibrating;
    private double calibrationTurnRad;
    private double calibrationErrorRadPerSec;

    // The true pose at the last track(), in RobotSim's field frame.
    private boolean tracking;
    private double lastTrueX;
    private double lastTrueY;
    private double lastTrueHeadingCwDeg;
    private long lastNanos;

    public SimPinpoint(LongSupplier clockNanos) {
        this.clockNanos = clockNanos;

        Mockito.doAnswer(call -> {
            xDirection = direction(call.getArgument(0));
            yDirection = direction(call.getArgument(1));
            return null;
        }).when(device).setEncoderDirections(any(), any());
        Mockito.doAnswer(call -> {
            DistanceUnit unit = call.getArgument(2);
            xOffsetInches = inches(call.getArgument(0), unit);
            yOffsetInches = inches(call.getArgument(1), unit);
            return null;
        }).when(device).setOffsets(anyDouble(), anyDouble(), any());
        Mockito.doAnswer(call -> { zero(); calibrate(); return null; }).when(device).resetPosAndIMU();
        Mockito.doAnswer(call -> { calibrate(); return null; }).when(device).recalibrateIMU();
        Mockito.doAnswer(call -> {
            Pose2D pose = call.getArgument(0);
            estX = pose.getX(DistanceUnit.INCH);
            estY = pose.getY(DistanceUnit.INCH);
            estHeadingRad = pose.getHeading(AngleUnit.RADIANS);
            return null;
        }).when(device).setPosition(any());
        Mockito.doAnswer(call -> { estX = inches(call.getArgument(0), call.getArgument(1)); return null; })
                .when(device).setPosX(anyDouble(), any());
        Mockito.doAnswer(call -> { estY = inches(call.getArgument(0), call.getArgument(1)); return null; })
                .when(device).setPosY(anyDouble(), any());
        Mockito.doAnswer(call -> {
            AngleUnit unit = call.getArgument(1);
            estHeadingRad = unit.toRadians(call.<Double>getArgument(0));
            return null;
        }).when(device).setHeading(anyDouble(), any());

        Mockito.doAnswer(call -> { latch(); return null; }).when(device).update();
        Mockito.doAnswer(call -> { latch(); return null; }).when(device).update(any());

        Mockito.when(device.getPosX(any())).thenAnswer(call ->
                call.<DistanceUnit>getArgument(0).fromMm(DistanceUnit.INCH.toMm(readX)));
        Mockito.when(device.getPosY(any())).thenAnswer(call ->
                call.<DistanceUnit>getArgument(0).fromMm(DistanceUnit.INCH.toMm(readY)));
        Mockito.when(device.getHeading(any(AngleUnit.class))).thenAnswer(call ->
                call.<AngleUnit>getArgument(0).fromRadians(readHeadingRad));
        Mockito.when(device.getHeading(any(UnnormalizedAngleUnit.class))).thenAnswer(call ->
                call.<UnnormalizedAngleUnit>getArgument(0).fromRadians(readHeadingRad));
        Mockito.when(device.getPosition()).thenAnswer(call -> new Pose2D(DistanceUnit.INCH, readX, readY,
                AngleUnit.RADIANS, AngleUnit.normalizeRadians(readHeadingRad)));
        Mockito.when(device.getDeviceStatus()).thenAnswer(call -> calibrating
                ? GoBildaPinpointDriver.DeviceStatus.CALIBRATING
                : GoBildaPinpointDriver.DeviceStatus.READY);
        Mockito.when(device.getXOffset(any())).thenAnswer(call ->
                (float) call.<DistanceUnit>getArgument(0).fromMm(DistanceUnit.INCH.toMm(xOffsetInches)));
        Mockito.when(device.getYOffset(any())).thenAnswer(call ->
                (float) call.<DistanceUnit>getArgument(0).fromMm(DistanceUnit.INCH.toMm(yOffsetInches)));
    }

    /**
     * The robot has just been put down at the simulator's pose and the
     * Pinpoint zeroed there with the robot still - the state an OpMode's
     * init() finds on the real field. Throws away any calibration error.
     */
    public void placeAt(RobotSim sim) {
        zero();
        latch();
        calibrating = false;
        calibrationErrorRadPerSec = 0;
        tracking = true;
        lastTrueX = sim.xInches;
        lastTrueY = sim.yInches;
        lastTrueHeadingCwDeg = sim.headingDeg;
        lastNanos = clockNanos.getAsLong();
    }

    /** Follows the simulated robot since the last call, as the device does continuously. */
    public void track(RobotSim sim) {
        long now = clockNanos.getAsLong();
        if (!tracking) {
            placeAt(sim);
            return;
        }
        double dt = Math.max(0, (now - lastNanos) / 1e9);

        // True motion of the robot's centre, in its own frame (forward, left),
        // and its true turn, counterclockwise-positive.
        double dTurnCwDeg = wrapDeg(sim.headingDeg - lastTrueHeadingCwDeg);
        double midCw = Math.toRadians(lastTrueHeadingCwDeg + dTurnCwDeg / 2);
        double dx = sim.xInches - lastTrueX;
        double dy = sim.yInches - lastTrueY;
        double forward = dx * Math.sin(midCw) + dy * Math.cos(midCw);
        double left = -dx * Math.cos(midCw) + dy * Math.sin(midCw);
        double turn = -Math.toRadians(dTurnCwDeg);

        lastTrueX = sim.xInches;
        lastTrueY = sim.yInches;
        lastTrueHeadingCwDeg = sim.headingDeg;
        lastNanos = now;

        // The gyro of a board mounted upside down feels every turn the other way.
        int gyroSign = mounting.mirrored ? -1 : 1;
        if (calibrating) {
            // The gyro takes whatever it feels now as its zero.
            calibrationTurnRad += gyroSign * turn;
            if (now >= calibrationEndsNanos) {
                calibrating = false;
                calibrationErrorRadPerSec = -calibrationTurnRad / CALIBRATION_SECONDS;
            }
            return;
        }

        // What each pod rolls, where it is, the way it counts, and which port it is in.
        double forwardPod = mounting.forwardPodSign * (forward - turn * mounting.forwardPodLeftInches);
        double strafePod = mounting.strafePodSign * (left + turn * mounting.strafePodForwardInches);
        double xPod = mounting.mirrored ? strafePod : forwardPod;
        double yPod = mounting.mirrored ? forwardPod : strafePod;

        // What the device makes of it, with the OpMode's settings.
        double gyroTurn = gyroSign * (turn + Math.toRadians(mounting.gyroDriftDps) * dt)
                + calibrationErrorRadPerSec * dt;
        double estForward = xDirection * xPod + gyroTurn * xOffsetInches;
        double estLeft = yDirection * yPod - gyroTurn * yOffsetInches;
        double mid = estHeadingRad + gyroTurn / 2;
        estX += estForward * Math.cos(mid) - estLeft * Math.sin(mid);
        estY += estForward * Math.sin(mid) + estLeft * Math.cos(mid);
        estHeadingRad += gyroTurn;
    }

    private void zero() {
        estX = 0;
        estY = 0;
        estHeadingRad = 0;
    }

    private void calibrate() {
        calibrating = true;
        calibrationTurnRad = 0;
        calibrationEndsNanos = clockNanos.getAsLong() + (long) (CALIBRATION_SECONDS * 1e9);
    }

    private void latch() {
        readX = estX;
        readY = estY;
        readHeadingRad = estHeadingRad;
    }

    private static int direction(GoBildaPinpointDriver.EncoderDirection d) {
        return d == GoBildaPinpointDriver.EncoderDirection.REVERSED ? -1 : 1;
    }

    private static double inches(double value, DistanceUnit unit) {
        return DistanceUnit.INCH.fromMm(unit.toMm(value));
    }

    private static double wrapDeg(double deg) {
        double d = deg % 360.0;
        if (d > 180.0) d -= 360.0;
        if (d < -180.0) d += 360.0;
        return d;
    }
}

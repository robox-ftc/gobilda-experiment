package org.firstinspires.ftc.teamcode.calibration;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.devices.Drivetrain;
import org.firstinspires.ftc.teamcode.devices.DrivetrainControls;
import org.firstinspires.ftc.teamcode.devices.PinpointSetup;
import org.firstinspires.ftc.teamcode.utils.TelemetryLogger;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Calibration step 2: is the Pinpoint set up the way the robot really is?
 * It is set up and read exactly as {@link PinpointSetup} says, which is how
 * BIOBUZZAutoOdo uses it, so the answers apply to that auto.
 *
 * <p><b>In INIT</b> (no time limit), three guided checks. For each, move the
 * robot by hand - or drive it gently with gamepad1's sticks - then press A:
 * <ol>
 *   <li>forward about 24 in: the code must read forward;</li>
 *   <li>left about 24 in: the code must read left;</li>
 *   <li>counterclockwise about 90 degrees: the code must read counterclockwise.</li>
 * </ol>
 * Each is judged from the Pinpoint's raw X and Y ports and heading, so a
 * wrong result says which settings in PinpointSetup would make it right - a
 * pod direction, or whether the Pinpoint reads as a mirror image (MIRRORED) -
 * or that it cannot be fixed in code. B redoes the current check, DPAD DOWN
 * recalibrates the gyro (keep the robot still).
 *
 * <p><b>After START</b> the robot turns about 180 degrees on the spot by
 * itself, and back. Turning should not move the reported position; if it does,
 * the pod offsets are wrong, and how far it moved says by how much. Needs about
 * 2 ft clear around the robot, and working drive control. Skipped if the
 * directions are wrong. Everything is also written to a log file.
 */
@Autonomous(name = "Cal 2: Pinpoint Check", group = "Calibration")
public class CalPinpoint extends OpMode {

    /** A push shorter than this is taken as "didn't move", inches. */
    static final double MIN_PUSH_INCHES = 6;
    static final double MIN_TURN_DEGREES = 30;

    /** The on-the-spot turn after START. */
    static final double TURN_TARGET_DEGREES = 180;
    static final double TURN_P = 0.012;
    static final double TURN_MIN_POWER = 0.12;
    static final double TURN_MAX_POWER = 0.35;
    static final double TURN_TOLERANCE_DEGREES = 3;
    static final double TURN_TIMEOUT_SECONDS = 5;
    static final double SETTLE_SECONDS = 0.6;
    /** The two turns' offset estimates must agree this closely to be believed, inches. */
    static final double TURNS_AGREE_INCHES = 1.0;

    private enum Check { FORWARD, LEFT, TURN, DONE }

    private GoBildaPinpointDriver pinpoint;
    private Drivetrain drivetrain;
    private final DrivetrainControls controls = new DrivetrainControls();
    private TelemetryLogger log;

    private Check check = Check.FORWARD;
    private final List<String> initResults = new ArrayList<>();
    /** The raw readings each check ended on: the Pinpoint's X and Y ports (inches) and its heading (degrees). */
    private double forwardRawX, forwardRawY, leftRawX, leftRawY, turnRawHeading;
    /** Whether the code reads all three moves right; null until the checks are done. */
    private Boolean directionsOk;

    // After START: 0 = turn out, 1 = settle, 2 = turn back, 3 = settle, 4 = done.
    private int phase;
    private final ElapsedTime timer = new ElapsedTime();
    private double turnStartX, turnStartY, turnStartHeading, turnGoal;
    private final List<double[]> offsetEstimates = new ArrayList<>();
    private final List<String> runResults = new ArrayList<>();

    @Override
    public void init() {
        log = new TelemetryLogger(telemetry, "CalPinpoint");
        try {
            drivetrain = new Drivetrain(hardwareMap, telemetry, false);
        } catch (Exception e) {
            drivetrain = null;
        }
        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            PinpointSetup.configure(pinpoint);
            pinpoint.resetPosAndIMU();
        } catch (Exception e) {
            pinpoint = null;
        }
        log.addLine(String.format(Locale.US,
                "Code says: MIRRORED %s, X port %s, Y port %s, forward pod %.2f in left, sideways pod %.2f in forward",
                PinpointSetup.MIRRORED, PinpointSetup.X_POD_DIRECTION, PinpointSetup.Y_POD_DIRECTION,
                PinpointSetup.FORWARD_POD_LEFT_INCHES, PinpointSetup.STRAFE_POD_FORWARD_INCHES));
        log.update();
    }

    @Override
    public void init_loop() {
        if (pinpoint == null) {
            telemetry.addLine("NO PINPOINT in the robot configuration (name \"pinpoint\").");
            telemetry.update();
            return;
        }
        pinpoint.update();
        double rawX = pinpoint.getPosX(DistanceUnit.INCH);
        double rawY = pinpoint.getPosY(DistanceUnit.INCH);
        double rawHeading = pinpoint.getHeading(AngleUnit.DEGREES);

        // Gentle stick driving, for the test UI or for a robot too heavy to push.
        if (drivetrain != null) {
            controls.translationXPower = 0.3 * gamepad1.left_stick_x;
            controls.translationYPower = -0.3 * gamepad1.left_stick_y;
            controls.rotationPower = 0.3 * gamepad1.right_stick_x;
            drivetrain.run(controls);
        }

        if (gamepad1.dpadDownWasPressed() || gamepad2.dpadDownWasPressed()) {
            pinpoint.resetPosAndIMU();
        } else if (gamepad1.bWasPressed() || gamepad2.bWasPressed()) {
            zero();
        } else if ((gamepad1.aWasPressed() || gamepad2.aWasPressed()) && check != Check.DONE) {
            judge(rawX, rawY, rawHeading);
            zero();
        }

        switch (check) {
            case FORWARD:
                telemetry.addLine("CHECK 1 of 3: move the robot FORWARD about 24 in, then press A.");
                break;
            case LEFT:
                telemetry.addLine("CHECK 2 of 3: move the robot LEFT about 24 in (no turning), then press A.");
                break;
            case TURN:
                telemetry.addLine("CHECK 3 of 3: turn the robot COUNTERCLOCKWISE about 90 degrees, then press A.");
                break;
            case DONE:
                telemetry.addLine("Checks done. Press START for the offset check: the robot turns");
                telemetry.addLine("about 180 degrees on the spot and back. Needs 2 ft clear around it.");
                break;
        }
        telemetry.addLine("Push it by hand or drive gently with gamepad1. B = redo, DPAD DOWN = recalibrate gyro (keep still).");
        telemetry.addData("Pinpoint", pinpoint.getDeviceStatus());
        telemetry.addData("Code reads", "%.1f in forward, %.1f in left, %.1f deg CCW",
                PinpointSetup.forwardInches(pinpoint), PinpointSetup.leftInches(pinpoint),
                PinpointSetup.headingDegrees(pinpoint));
        telemetry.addData("Raw", "X %.1f in, Y %.1f in, heading %.1f deg", rawX, rawY, rawHeading);
        for (String line : initResults) telemetry.addLine(line);
        telemetry.update();
    }

    /** Records and reports the move since the last zero, for the current check. */
    private void judge(double rawX, double rawY, double rawHeading) {
        double forward = PinpointSetup.forwardInches(pinpoint);
        double left = PinpointSetup.leftInches(pinpoint);
        double ccw = PinpointSetup.headingDegrees(pinpoint);
        String raw = String.format(Locale.US, " (raw X %.1f, Y %.1f)", rawX, rawY);
        String line;
        switch (check) {
            case FORWARD:
                if (Math.hypot(rawX, rawY) < MIN_PUSH_INCHES) {
                    line = "Forward: hardly moved" + raw + ". Try again, further.";
                    break;
                }
                forwardRawX = rawX;
                forwardRawY = rawY;
                line = String.format(Locale.US, "Forward: code read %.1f in forward, %.1f in left%s. %s",
                        forward, left, raw, forward > Math.abs(left) ? "OK. (Check against your tape measure.)" : "WRONG.");
                check = Check.LEFT;
                break;
            case LEFT:
                if (Math.hypot(rawX, rawY) < MIN_PUSH_INCHES) {
                    line = "Left: hardly moved" + raw + ". Try again, further.";
                    break;
                }
                leftRawX = rawX;
                leftRawY = rawY;
                line = String.format(Locale.US, "Left: code read %.1f in left, %.1f in forward%s. %s",
                        left, forward, raw, left > Math.abs(forward) ? "OK." : "WRONG.");
                check = Check.TURN;
                break;
            case TURN:
                if (Math.abs(rawHeading) < MIN_TURN_DEGREES) {
                    line = String.format(Locale.US, "Turn: hardly turned (raw heading %.1f deg). Try again, further.",
                            rawHeading);
                    break;
                }
                turnRawHeading = rawHeading;
                line = String.format(Locale.US, "Turn: code read %.1f deg counterclockwise (raw heading %.1f). %s",
                        ccw, rawHeading, ccw > 0 ? "OK." : "WRONG.");
                check = Check.DONE;
                initResults.add(line);
                log.addLine(line);
                log.update();
                List<String> summary = diagnose();
                initResults.addAll(0, summary);
                for (String s : summary) {
                    log.addLine(s);
                    log.update();
                }
                return;
            default:
                return;
        }
        initResults.add(line);
        log.addLine(line);
        log.update();
    }

    /**
     * What the three checks say about the setup, from the raw readings.
     *
     * <p>Fitted as goBILDA intends, the X port's pod measures forward, the Y
     * port's measures left, and heading is counterclockwise. Fitted as a mirror
     * image (MIRRORED), the X port measures right, the Y port backward, and
     * heading is clockwise - which the code can undo. Anything else (pods
     * swapped but the board right side up, or the other way round) it cannot.
     */
    List<String> diagnose() {
        List<String> out = new ArrayList<>();
        boolean forwardOk = PinpointSetup.MIRRORED ? -forwardRawY > Math.abs(forwardRawX) : forwardRawX > Math.abs(forwardRawY);
        boolean leftOk = PinpointSetup.MIRRORED ? -leftRawX > Math.abs(leftRawY) : leftRawY > Math.abs(leftRawX);
        boolean turnOk = PinpointSetup.MIRRORED ? turnRawHeading < 0 : turnRawHeading > 0;
        directionsOk = forwardOk && leftOk && turnOk;
        if (directionsOk) {
            out.add("DIRECTIONS: all correct. The test UI's default Pinpoint matches this robot.");
            return out;
        }

        boolean forwardOnY = Math.abs(forwardRawY) > Math.abs(forwardRawX);
        boolean leftOnY = Math.abs(leftRawY) > Math.abs(leftRawX);
        boolean headingClockwise = turnRawHeading < 0;
        if (forwardOnY == leftOnY) {
            out.add("DIRECTIONS WRONG: forward and left both showed on the Pinpoint's " + (forwardOnY ? "Y" : "X")
                    + " port. One pod is not counting, or both measure the same way. Check the pods and their cables.");
            return out;
        }
        if (forwardOnY != headingClockwise) {
            out.add(forwardOnY
                    ? "DIRECTIONS WRONG: the pods are in each other's ports but the Pinpoint is right side up. The code"
                            + " cannot fix that: swap the two pod cables, then run this again."
                    : "DIRECTIONS WRONG: the Pinpoint is upside down but the pods are in their usual ports. The code"
                            + " cannot fix that: remount it label side up (or swap the two pod cables), then run this again.");
            return out;
        }

        boolean mirrored = forwardOnY;
        // Mirrored, the X port must count the robot moving right and the Y port moving backward.
        boolean xCountsRight = mirrored ? leftRawX < 0 : forwardRawX > 0;
        boolean yCountsRight = mirrored ? forwardRawY < 0 : leftRawY > 0;
        EncoderDirection x = xCountsRight ? PinpointSetup.X_POD_DIRECTION : flip(PinpointSetup.X_POD_DIRECTION);
        EncoderDirection y = yCountsRight ? PinpointSetup.Y_POD_DIRECTION : flip(PinpointSetup.Y_POD_DIRECTION);
        List<String> fixes = new ArrayList<>();
        if (mirrored != PinpointSetup.MIRRORED) fixes.add("MIRRORED = " + mirrored);
        if (x != PinpointSetup.X_POD_DIRECTION) fixes.add("X_POD_DIRECTION = " + x);
        if (y != PinpointSetup.Y_POD_DIRECTION) fixes.add("Y_POD_DIRECTION = " + y);
        out.add("DIRECTIONS WRONG. FIX in devices/PinpointSetup: " + String.join(", ", fixes)
                + ". The test UI's default Pinpoint then matches this robot too.");
        return out;
    }

    @Override
    public void start() {
        log.startClock();
        if (pinpoint == null || drivetrain == null) {
            runResults.add(pinpoint == null ? "No Pinpoint: nothing to check." : "No drivetrain: cannot turn.");
            phase = 4;
            return;
        }
        if (Boolean.FALSE.equals(directionsOk)) {
            runResults.add("Offset check skipped: fix the directions first (results from INIT below).");
            phase = 4;
            return;
        }
        if (directionsOk == null) {
            runResults.add("NOTE: the direction checks in INIT were not all done, so the offsets below assume"
                    + " the directions are right.");
        }
        pinpoint.update();
        beginTurn(TURN_TARGET_DEGREES);
    }

    @Override
    public void loop() {
        if (phase < 4) {
            pinpoint.update();
            double h = PinpointSetup.headingDegrees(pinpoint);
            switch (phase) {
                case 0:
                case 2:
                    double error = AngleUnit.normalizeDegrees(turnGoal - h);
                    if (Math.abs(error) < TURN_TOLERANCE_DEGREES || timer.seconds() > TURN_TIMEOUT_SECONDS) {
                        setTurn(0);
                        phase++;
                        timer.reset();
                    } else {
                        // Drivetrain rotation is clockwise-positive; heading is counterclockwise-positive.
                        double power = Math.max(TURN_MIN_POWER, Math.min(TURN_MAX_POWER, Math.abs(error) * TURN_P));
                        setTurn(-Math.signum(error) * power);
                    }
                    break;
                case 1:
                case 3:
                    setTurn(0);
                    if (timer.seconds() >= SETTLE_SECONDS) {
                        endTurn();
                        if (phase == 1) {
                            beginTurn(0);
                            phase = 2;
                        } else {
                            summarizeOffsets();
                            phase = 4;
                        }
                    }
                    break;
            }
            if (phase < 4) {
                telemetry.addLine(phase <= 1 ? "Turning about 180 degrees..." : "Turning back...");
                telemetry.addData("Heading", "%.1f deg", h);
                telemetry.addData("Forward", "%.2f in", PinpointSetup.forwardInches(pinpoint));
                telemetry.addData("Left", "%.2f in", PinpointSetup.leftInches(pinpoint));
                log.update();
                return;
            }
        }
        if (drivetrain != null) setTurn(0);
        telemetry.addLine("DONE (also in the log file).");
        for (String line : runResults) telemetry.addLine(line);
        for (String line : initResults) telemetry.addLine(line);
        telemetry.update();
    }

    @Override
    public void stop() {
        if (drivetrain != null) setTurn(0);
        if (log != null) log.close();
    }

    private void beginTurn(double goalDegrees) {
        turnStartX = PinpointSetup.forwardInches(pinpoint);
        turnStartY = PinpointSetup.leftInches(pinpoint);
        turnStartHeading = PinpointSetup.headingDegrees(pinpoint);
        turnGoal = goalDegrees;
        timer.reset();
    }

    /**
     * Works out the offset error from one finished turn.
     *
     * <p>If the offsets are wrong, turning makes the reported centre move as if
     * a point {@code u} off the true centre were being tracked (in the robot's
     * frame: u.x along X, u.y along Y). Turning by T from heading h0 moves that
     * point by R(h0) * M(T) * u, with M(T) = [[sin T, cos T - 1], [1 - cos T, sin T]].
     * Solve for u from the measured move and turn, in the robot's terms (as
     * PinpointSetup reads them). Then, by how the Pinpoint uses its offsets,
     * u.x = (forward pod's left, as set) - (as it really is) and
     * u.y = (sideways pod's forward, as it really is) - (as set). That holds
     * mirrored too: mirroring the device and mirroring its readings back
     * cancel out.
     */
    private void endTurn() {
        double dx = PinpointSetup.forwardInches(pinpoint) - turnStartX;
        double dy = PinpointSetup.leftInches(pinpoint) - turnStartY;
        double turned = AngleUnit.normalizeDegrees(PinpointSetup.headingDegrees(pinpoint) - turnStartHeading);
        double[] u = offsetError(dx, dy, turnStartHeading, turned);
        String line = String.format(Locale.US, "Turned %.1f deg; reported centre moved %.2f in forward, %.2f in left",
                turned, dx, dy);
        if (u == null) {
            line += " - too small a turn to work out the offsets.";
        } else {
            offsetEstimates.add(u);
        }
        runResults.add(line);
        log.addLine(line);
        log.update();
    }

    /** Solves for u; null if the turn was too small (or a full circle) to tell. Package-private for tests. */
    static double[] offsetError(double dx, double dy, double startHeadingDeg, double turnedDeg) {
        double t = Math.toRadians(turnedDeg);
        if (Math.abs(turnedDeg) < 60) return null;
        double h0 = Math.toRadians(startHeadingDeg);
        // Into the frame the robot faced at the start of the turn.
        double rx = dx * Math.cos(h0) + dy * Math.sin(h0);
        double ry = -dx * Math.sin(h0) + dy * Math.cos(h0);
        double s = Math.sin(t);
        double c = Math.cos(t);
        double det = s * s + (1 - c) * (1 - c);
        // Inverse of [[s, c-1], [1-c, s]] is [[s, 1-c], [c-1, s]] / det.
        return new double[]{(s * rx + (1 - c) * ry) / det, ((c - 1) * rx + s * ry) / det};
    }

    private void summarizeOffsets() {
        if (offsetEstimates.isEmpty()) {
            runResults.add(0, "OFFSETS: could not measure (the robot did not turn far enough).");
            log.addLine(runResults.get(0));
            log.update();
            return;
        }
        if (offsetEstimates.size() >= 2) {
            double[] a = offsetEstimates.get(0);
            double[] b = offsetEstimates.get(1);
            if (Math.abs(a[0] - b[0]) > TURNS_AGREE_INCHES || Math.abs(a[1] - b[1]) > TURNS_AGREE_INCHES) {
                runResults.add(0, "OFFSETS: NOT MEASURED. The two turns disagree, so the robot was pushed or bumped"
                        + " while turning (backed against a wall? touching a field element?). Put it on open floor,"
                        + " 2 ft clear all round, and run this again.");
                log.addLine(runResults.get(0));
                log.update();
                return;
            }
        }
        double ux = 0, uy = 0;
        for (double[] u : offsetEstimates) {
            ux += u[0];
            uy += u[1];
        }
        ux /= offsetEstimates.size();
        uy /= offsetEstimates.size();
        double forwardPodLeft = PinpointSetup.FORWARD_POD_LEFT_INCHES - ux;
        double strafePodForward = PinpointSetup.STRAFE_POD_FORWARD_INCHES + uy;

        List<String> out = new ArrayList<>();
        if (Math.abs(ux) < 0.5 && Math.abs(uy) < 0.5) {
            out.add(String.format(Locale.US, "OFFSETS: within half an inch of the code's (%.2f, %.2f). OK.",
                    PinpointSetup.FORWARD_POD_LEFT_INCHES, PinpointSetup.STRAFE_POD_FORWARD_INCHES));
        } else {
            out.add(String.format(Locale.US, "OFFSETS look like FORWARD_POD_LEFT_INCHES = %.1f,"
                    + " STRAFE_POD_FORWARD_INCHES = %.1f (code has %.1f, %.1f). Check with a ruler before changing.",
                    forwardPodLeft, strafePodForward,
                    PinpointSetup.FORWARD_POD_LEFT_INCHES, PinpointSetup.STRAFE_POD_FORWARD_INCHES));
            out.add(String.format(Locale.US, "Test UI: run with --pinpoint=xpod=%.1f,ypod=%.1f to give it this"
                    + " robot's Pinpoint.", forwardPodLeft, strafePodForward));
        }
        out.add("(A mecanum robot never turns exactly on the spot, so expect about half an inch of noise.)");
        runResults.addAll(0, out);
        for (String line : out) {
            log.addLine(line);
            log.update();
        }
    }

    private void setTurn(double power) {
        controls.translationXPower = 0;
        controls.translationYPower = 0;
        controls.rotationPower = power;
        drivetrain.run(controls);
    }

    private void zero() {
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
    }

    private static GoBildaPinpointDriver.EncoderDirection flip(GoBildaPinpointDriver.EncoderDirection d) {
        return d == GoBildaPinpointDriver.EncoderDirection.FORWARD
                ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                : GoBildaPinpointDriver.EncoderDirection.FORWARD;
    }

    private static String fmt(double v) {
        return String.format(Locale.US, "%.1f", v);
    }
}

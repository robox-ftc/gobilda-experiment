package org.firstinspires.ftc.teamcode.calibration;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.devices.BatteryCompensation;
import org.firstinspires.ftc.teamcode.devices.Drivetrain;
import org.firstinspires.ftc.teamcode.devices.DrivetrainControls;
import org.firstinspires.ftc.teamcode.devices.PinpointSetup;
import org.firstinspires.ftc.teamcode.utils.TelemetryLogger;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Calibration step 3: how fast does the robot really go for a given power?
 * Run it on the robot <em>and</em> in the test UI and compare the two result
 * screens (or log files) line by line.
 *
 * <p>Timed moves at fixed power, sent through {@link Drivetrain} exactly as
 * the time-based auto sends them, measured with the Pinpoint: forward and back,
 * right and left, and turning, each at 0.3 and 0.6 power for 1 second, in
 * out-and-back pairs so the robot ends near where it started. About 21
 * seconds. <b>Needs about 5 ft clear in front, behind and to each side</b>.
 *
 * <p>Powers are corrected for the battery as BIOBUZZAuto corrects them
 * ({@link BatteryCompensation}), so the speeds are the ones at 12 V.
 *
 * <p>For each move it reports the steady speed (measured over the second half
 * of the move, after speeding up) and the whole distance including the stop.
 * From those: full-power speed forward, sideways and turning, which are
 * BIOBUZZAuto's FORWARD_IPS, STRAFE_IPS and TURN_DPS, and go into the test
 * UI's {@code RobotSim}; and whether speed follows power at all.
 *
 * <p>The drive runs in {@link Drivetrain#DRIVE_RUN_MODE}, as the autos do.
 * In INIT, DPAD UP switches it to RUN_USING_ENCODER for comparison - which
 * only works with the drive encoders connected (Cal 1).
 *
 * <p>Run Cal 2 first: if the Pinpoint reads wrong, so will these.
 */
@Autonomous(name = "Cal 3: Drive Speed", group = "Calibration")
public class CalDriveSpeed extends OpMode {

    static final double MOVE_SECONDS = 1.0;
    /** Steady speed is measured from here to the end of the move (after speeding up). */
    static final double STEADY_FROM_SECONDS = 0.5;
    static final double SETTLE_SECONDS = 0.75;
    /** An out-and-back pair whose distances or speeds differ by more than this fraction was blocked. */
    static final double PAIR_TOLERANCE = 0.15;
    /** A turn that moves the robot further than this was probably pushed by a wall, inches. */
    static final double TURN_DRIFT_INCHES = 1.0;

    enum Axis { FORWARD, LEFT, TURN }

    /** One timed move: drivetrain controls as the autos use them (x right, y forward, rotation clockwise). */
    static final class Move {
        final String name;
        final Axis axis;
        final double x, y, r;
        /** The power, positive, for working out speed per power. */
        final double power;
        /** +1 if this move should make its axis reading go up, -1 if down. */
        final int sign;

        Move(String name, Axis axis, double x, double y, double r, double power, int sign) {
            this.name = name;
            this.axis = axis;
            this.x = x;
            this.y = y;
            this.r = r;
            this.power = power;
            this.sign = sign;
        }
    }

    static final Move[] MOVES = {
            new Move("forward 0.3", Axis.FORWARD, 0, 0.3, 0, 0.3, +1),
            new Move("back 0.3", Axis.FORWARD, 0, -0.3, 0, 0.3, -1),
            new Move("forward 0.6", Axis.FORWARD, 0, 0.6, 0, 0.6, +1),
            new Move("back 0.6", Axis.FORWARD, 0, -0.6, 0, 0.6, -1),
            new Move("right 0.3", Axis.LEFT, 0.3, 0, 0, 0.3, -1),
            new Move("left 0.3", Axis.LEFT, -0.3, 0, 0, 0.3, +1),
            new Move("right 0.6", Axis.LEFT, 0.6, 0, 0, 0.6, -1),
            new Move("left 0.6", Axis.LEFT, -0.6, 0, 0, 0.6, +1),
            new Move("turn CCW 0.3", Axis.TURN, 0, 0, -0.3, 0.3, +1),
            new Move("turn CW 0.3", Axis.TURN, 0, 0, 0.3, 0.3, -1),
            new Move("turn CCW 0.6", Axis.TURN, 0, 0, -0.6, 0.6, +1),
            new Move("turn CW 0.6", Axis.TURN, 0, 0, 0.6, 0.6, -1),
    };

    private Drivetrain drivetrain;
    private final DrivetrainControls controls = new DrivetrainControls();
    private GoBildaPinpointDriver pinpoint;
    private TelemetryLogger log;
    private boolean usingEncoders;
    private BatteryCompensation battery;

    private int index;
    private boolean settling;
    private boolean done;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean steadyMarked;
    private double[] startPose;
    private double steadyTime;
    private double endTime;
    private double[] steadyPose;
    private double[] endPose;

    /** Per move: steady speed along its axis (in/s or deg/s, positive = the commanded way). */
    final double[] steadySpeed = new double[MOVES.length];
    /** Per move: whole distance along its axis including the stop (in or deg, positive = the commanded way). */
    final double[] distance = new double[MOVES.length];
    /** Per move: drift off its axis (in sideways for a straight move, deg of turn for a straight move, in for a turn). */
    final double[] drift = new double[MOVES.length];
    /** Per move: left out of the results, because its out-and-back pair did not match. */
    final boolean[] blocked = new boolean[MOVES.length];
    private final List<String> results = new ArrayList<>();

    @Override
    public void init() {
        log = new TelemetryLogger(telemetry, "CalDriveSpeed");
        drivetrain = new Drivetrain(hardwareMap, telemetry, false);
        battery = new BatteryCompensation(hardwareMap);
        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            PinpointSetup.configure(pinpoint);
            pinpoint.resetPosAndIMU();
        } catch (Exception e) {
            pinpoint = null;
        }
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpadUpWasPressed() || gamepad2.dpadUpWasPressed()) usingEncoders = !usingEncoders;
        telemetry.addLine("Needs about 5 ft clear in front, behind and to each side.");
        telemetry.addLine("12 timed moves, 1 s each, out and back. About 21 s. STOP at any time.");
        telemetry.addData("Drive motor mode (DPAD UP to switch)", usingEncoders
                ? "RUN_USING_ENCODER (comparison: needs the encoders connected)"
                : Drivetrain.DRIVE_RUN_MODE + " (as the autos)");
        if (!Double.isNaN(battery.volts())) telemetry.addData("Battery", "%.2f V", battery.volts());
        if (pinpoint == null) {
            telemetry.addLine("NO PINPOINT: the moves still run; measure them with a tape measure.");
        } else {
            pinpoint.update();
            telemetry.addData("Pinpoint", pinpoint.getDeviceStatus());
        }
        telemetry.update();
    }

    @Override
    public void start() {
        log.startClock();
        DcMotor.RunMode mode = usingEncoders ? DcMotor.RunMode.RUN_USING_ENCODER : Drivetrain.DRIVE_RUN_MODE;
        for (String name : CalDriveMotors.NAMES) {
            try {
                hardwareMap.get(DcMotorEx.class, name).setMode(mode);
            } catch (Exception ignored) {
                // Drivetrain would already have failed without it.
            }
        }
        log.addLine("Drive motors in " + mode + (Double.isNaN(battery.volts()) ? ", no battery reading"
                : String.format(Locale.US, ", battery %.2f V at START", battery.volts())));
        if (pinpoint != null) {
            pinpoint.update();
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        }
        index = 0;
        beginMove();
    }

    @Override
    public void loop() {
        if (done) {
            showResults();
            return;
        }
        if (pinpoint != null) pinpoint.update();
        Move move = MOVES[index];

        if (!settling) {
            if (!steadyMarked && timer.seconds() >= STEADY_FROM_SECONDS) {
                steadyPose = pose();
                steadyTime = timer.seconds();
                steadyMarked = true;
            }
            if (timer.seconds() >= MOVE_SECONDS) {
                endPose = pose();
                endTime = timer.seconds();
                drive(0, 0, 0);
                settling = true;
                timer.reset();
            } else {
                drive(move.x, move.y, move.r);
            }
        } else {
            drive(0, 0, 0);
            if (timer.seconds() >= SETTLE_SECONDS) {
                record(index, pose());
                index++;
                if (index >= MOVES.length) {
                    finish();
                    return;
                }
                beginMove();
            }
        }
        telemetry.addData("Move", "%d of %d: %s%s", index + 1, MOVES.length, MOVES[index].name,
                settling ? " (stopping)" : "");
        double[] p = pose();
        telemetry.addData("Pinpoint", "X %.1f in, Y %.1f in, heading %.1f deg", p[0], p[1], p[2]);
        log.update();
    }

    @Override
    public void stop() {
        if (drivetrain != null) drive(0, 0, 0);
        if (log != null) log.close();
    }

    private void beginMove() {
        settling = false;
        steadyMarked = false;
        startPose = pose();
        timer.reset();
    }

    /** X forward, Y left (inches), heading counterclockwise (degrees). Zeros without a Pinpoint. */
    private double[] pose() {
        if (pinpoint == null) return new double[3];
        return new double[]{PinpointSetup.forwardInches(pinpoint), PinpointSetup.leftInches(pinpoint),
                PinpointSetup.headingDegrees(pinpoint)};
    }

    /** What happened in move i, in the frame the robot faced when it began. */
    private void record(int i, double[] finalPose) {
        Move move = MOVES[i];
        double[] steady = along(move, steadyPose, endPose);
        double[] whole = along(move, startPose, finalPose);
        steadySpeed[i] = move.sign * steady[0] / Math.max(endTime - steadyTime, 1e-3);
        distance[i] = move.sign * whole[0];
        drift[i] = whole[1];
        String unit = move.axis == Axis.TURN ? "deg" : "in";
        String driftWhat = move.axis == Axis.TURN ? "in moved" : "deg turned";
        String line = String.format(Locale.US, "%-13s steady %6.1f %s/s, whole move %6.1f %s, drift %5.1f %s",
                move.name, steadySpeed[i], unit, distance[i], unit, drift[i], driftWhat);
        log.addLine(line);
        log.update();
        results.add(line);
    }

    /**
     * From pose a to pose b: [movement along the move's axis, drift]. For a
     * straight move, both in the robot's frame at a, and drift is the turn in
     * degrees; for a turn, the turn in degrees and the distance moved.
     */
    private double[] along(Move move, double[] a, double[] b) {
        double dx = b[0] - a[0];
        double dy = b[1] - a[1];
        double turned = AngleUnit.normalizeDegrees(b[2] - a[2]);
        double h = Math.toRadians(a[2]);
        double forward = dx * Math.cos(h) + dy * Math.sin(h);
        double left = -dx * Math.sin(h) + dy * Math.cos(h);
        switch (move.axis) {
            case FORWARD: return new double[]{forward, turned};
            case LEFT: return new double[]{left, turned};
            default: return new double[]{turned, Math.hypot(dx, dy)};
        }
    }

    private void finish() {
        done = true;
        drive(0, 0, 0);
        if (pinpoint == null) {
            results.add("No Pinpoint: compare your tape-measured distances with the test UI's.");
        } else {
            findBlocked();
            summarize(Axis.FORWARD, "Forward", "in/s");
            summarize(Axis.LEFT, "Sideways", "in/s");
            summarize(Axis.TURN, "Turning", "deg/s");
            double forward = fullPower(Axis.FORWARD);
            double sideways = fullPower(Axis.LEFT);
            if (forward > 0 && !Double.isNaN(sideways)) {
                results.add(String.format(Locale.US, "Sideways is %.0f%% of forward speed.", 100 * sideways / forward));
            }
            double turning = fullPower(Axis.TURN);
            if (!Double.isNaN(forward) && !Double.isNaN(sideways) && !Double.isNaN(turning)) {
                results.add(String.format(Locale.US, "For BIOBUZZAuto: FORWARD_IPS = %.1f, STRAFE_IPS = %.1f,"
                        + " TURN_DPS = %.0f", forward, sideways, turning));
            }
            results.add("Test UI (RobotSim): MAX_SPEED_IPS is the forward number, STRAFE_FRACTION the sideways"
                    + " percentage, MAX_TURN_DPS the turning number. Run this OpMode there to see its own.");
        }
        log.addLine("========== RESULTS ==========");
        log.update();
        for (String line : results) {
            log.addLine(line);
            log.update();
        }
    }

    /**
     * The moves come in out-and-back pairs at the same power, which should go
     * the same distance at the same speed. A pair that does not was stopped
     * or pushed by something - a wall, a field element - so it is left out of
     * the numbers rather than passed off as the robot's speed.
     */
    private void findBlocked() {
        for (int i = 0; i + 1 < MOVES.length; i += 2) {
            if (!mismatched(distance[i], distance[i + 1]) && !mismatched(steadySpeed[i], steadySpeed[i + 1])) continue;
            blocked[i] = true;
            blocked[i + 1] = true;
            String unit = MOVES[i].axis == Axis.TURN ? "deg" : "in";
            results.add(String.format(Locale.US, "BLOCKED: %s went %.1f %s but %s went %.1f %s. Something stopped or"
                            + " pushed the robot (a wall? a field element?). Left out of the numbers below; rerun"
                            + " with more room.",
                    MOVES[i].name, distance[i], unit, MOVES[i + 1].name, distance[i + 1], unit));
        }
        for (int i = 0; i < MOVES.length; i++) {
            if (MOVES[i].axis == Axis.TURN && Math.abs(drift[i]) > TURN_DRIFT_INCHES) {
                results.add(String.format(Locale.US, "NOTE: %s also moved %.1f in. Turning on the spot should not move"
                        + " the robot; was it backed against a wall?", MOVES[i].name, drift[i]));
            }
        }
    }

    /** True when two readings that should match differ by more than PAIR_TOLERANCE, or point opposite ways. */
    static boolean mismatched(double a, double b) {
        if (a * b < 0) return true;
        double big = Math.max(Math.abs(a), Math.abs(b));
        return big > 1e-6 && Math.abs(Math.abs(a) - Math.abs(b)) / big > PAIR_TOLERANCE;
    }

    /** Average steady speed divided by power, over the unblocked moves on one axis: the speed at full power. */
    double fullPower(Axis axis) {
        double sum = 0;
        int n = 0;
        for (int i = 0; i < MOVES.length; i++) {
            if (MOVES[i].axis != axis || blocked[i]) continue;
            sum += steadySpeed[i] / MOVES[i].power;
            n++;
        }
        return n == 0 ? Double.NaN : sum / n;
    }

    /**
     * Steady speed at 0.6 over steady speed at 0.3, for one axis: about 2 if
     * speed follows power. NaN if either power was blocked.
     */
    double powerRatio(Axis axis) {
        double low = 0, high = 0;
        int nLow = 0, nHigh = 0;
        for (int i = 0; i < MOVES.length; i++) {
            if (MOVES[i].axis != axis || blocked[i]) continue;
            if (MOVES[i].power < 0.45) {
                low += steadySpeed[i];
                nLow++;
            } else {
                high += steadySpeed[i];
                nHigh++;
            }
        }
        if (nLow == 0 || nHigh == 0) return Double.NaN;
        return (high / nHigh) / Math.max(low / nLow, 1e-6);
    }

    private void summarize(Axis axis, String what, String unit) {
        double full = fullPower(axis);
        double ratio = powerRatio(axis);
        if (Double.isNaN(full)) {
            results.add(what + ": NOT MEASURED, every move was blocked. Rerun with more room.");
            return;
        }
        String line = Double.isNaN(ratio)
                ? String.format(Locale.US, "%s at full power: %.1f %s (from one power only; the other was blocked)",
                        what, full, unit)
                : String.format(Locale.US, "%s at full power: %.1f %s (0.6 power ran %.1fx as fast as 0.3)",
                        what, full, unit, ratio);
        if (!Double.isNaN(ratio) && ratio < 1.5) {
            line += ". SPEED DOES NOT FOLLOW POWER: the drive motors are not holding the speed asked for."
                    + " Run Cal 1 (drive encoders), and check the drive runs in RUN_WITHOUT_ENCODER.";
        } else if (full <= 0) {
            line += ". WENT THE WRONG WAY: check the Pinpoint with Cal 2.";
        }
        results.add(line);
    }

    private void showResults() {
        drive(0, 0, 0);
        telemetry.addLine("DONE. Results (also in the log file):");
        for (String line : results) telemetry.addLine(line);
        telemetry.update();
    }

    private void drive(double x, double y, double r) {
        double k = battery.factor();
        controls.translationXPower = x * k;
        controls.translationYPower = y * k;
        controls.rotationPower = r * k;
        drivetrain.run(controls);
    }
}

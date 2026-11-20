package org.firstinspires.ftc.teamcode.calibration;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.TelemetryLogger;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Calibration step 1: are the four drive motors and their encoders wired right?
 *
 * <p><b>Prop the robot up so all four wheels are off the floor</b>, then START.
 * Each drive motor runs on its own, at 0.2 power and then 0.5, in the same
 * RUN_USING_ENCODER mode {@code Drivetrain} uses, while all four encoders are
 * watched. Takes about 14 seconds.
 *
 * <p>Why: in RUN_USING_ENCODER the hub holds each motor at a speed it reads from
 * the encoder plugged into that motor's port. If that encoder is unplugged,
 * broken or belongs to another motor, the hub never sees the speed it wants and
 * drives the motor at full power, whatever power the code asked for. The
 * time-based auto then goes several times further than the test UI shows.
 *
 * <p>For each motor the result says one of:
 * <ul>
 *   <li>OK;</li>
 *   <li>NO ENCODER COUNTS - the encoder cable is unplugged or broken;</li>
 *   <li>ENCODER IN THE WRONG PORT - another motor's encoder counted instead;</li>
 *   <li>COUNTS BACKWARDS;</li>
 *   <li>SPEED DOES NOT FOLLOW POWER - 0.5 power is not about 2.5x as fast as 0.2;</li>
 *   <li>SPEED OFF - it runs well away from what the configured motor type should.</li>
 * </ul>
 * Everything is also written to a log file (see {@link TelemetryLogger}).
 */
@Autonomous(name = "Cal 1: Drive Motors (wheels up)", group = "Calibration")
public class CalDriveMotors extends OpMode {

    /** The drive motors, by their names in the robot configuration (as in Drivetrain). */
    static final String[] NAMES = {"lf", "rf", "lb", "rb"};
    static final double[] POWERS = {0.2, 0.5};

    /** Each power runs this long; speed is measured over the last MEASURE_SECONDS of it. */
    static final double RUN_SECONDS = 1.5;
    static final double MEASURE_SECONDS = 1.0;
    static final double REST_SECONDS = 0.5;

    /** Below this, an encoder is taken as not counting (ticks per second). */
    static final double STILL_TICKS_PER_SEC = 30;

    private final DcMotorEx[] motors = new DcMotorEx[NAMES.length];
    private final List<String> missing = new ArrayList<>();
    private TelemetryLogger log;
    private final ElapsedTime timer = new ElapsedTime();

    // Which motor and power are running: step = motor * (POWERS.length + 1) + power index;
    // the extra slot per motor is the rest after it.
    private int step;
    private boolean done;
    private boolean measuring;
    private final int[] measureStart = new int[NAMES.length];

    /** speed[motor run][power][encoder read] in ticks per second. */
    private final double[][][] speed = new double[NAMES.length][POWERS.length][NAMES.length];
    private final List<String> results = new ArrayList<>();

    @Override
    public void init() {
        log = new TelemetryLogger(telemetry, "CalDriveMotors");
        for (int i = 0; i < NAMES.length; i++) {
            try {
                motors[i] = hardwareMap.get(DcMotorEx.class, NAMES[i]);
                motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motors[i].setPower(0);
            } catch (Exception e) {
                motors[i] = null;
                missing.add(NAMES[i]);
            }
        }
        for (int i = 0; i < NAMES.length; i++) {
            if (motors[i] != null) {
                log.addLine(NAMES[i] + " configured as: " + describeType(motors[i]));
                log.update();
            }
        }
    }

    @Override
    public void init_loop() {
        telemetry.addLine("PROP THE ROBOT UP: all four wheels off the floor.");
        telemetry.addLine("Each drive motor runs alone at 0.2 then 0.5 power (about 14 s).");
        telemetry.addLine("Watch which wheel turns: it should be the one named on screen.");
        if (!missing.isEmpty()) telemetry.addLine("NOT IN THE CONFIGURATION: " + missing);
        for (int i = 0; i < NAMES.length; i++) {
            if (motors[i] != null) {
                telemetry.addData(NAMES[i] + " encoder", motors[i].getCurrentPosition());
            }
        }
        telemetry.update();
    }

    @Override
    public void start() {
        log.startClock();
        step = 0;
        beginStep();
    }

    @Override
    public void loop() {
        if (done) {
            showResults();
            return;
        }
        int motor = step / (POWERS.length + 1);
        int power = step % (POWERS.length + 1);
        boolean resting = power == POWERS.length;
        double length = resting ? REST_SECONDS : RUN_SECONDS;

        if (!resting && !measuring && timer.seconds() >= RUN_SECONDS - MEASURE_SECONDS) {
            // Spun up: start counting.
            for (int i = 0; i < NAMES.length; i++) measureStart[i] = position(i);
            measuring = true;
        }
        if (timer.seconds() >= length) {
            if (!resting) {
                for (int i = 0; i < NAMES.length; i++) {
                    speed[motor][power][i] = (position(i) - measureStart[i]) / MEASURE_SECONDS;
                }
                log.addLine(String.format(Locale.US, "%s at %.1f power: encoders lf %.0f, rf %.0f, lb %.0f, rb %.0f ticks/s",
                        NAMES[motor], POWERS[power], speed[motor][power][0], speed[motor][power][1],
                        speed[motor][power][2], speed[motor][power][3]));
            }
            step++;
            if (step >= NAMES.length * (POWERS.length + 1)) {
                finish();
                return;
            }
            beginStep();
            motor = step / (POWERS.length + 1);
            power = step % (POWERS.length + 1);
            resting = power == POWERS.length;
        }

        telemetry.addLine(resting
                ? "Resting after " + NAMES[motor]
                : String.format(Locale.US, "Running %s at %.1f power: that wheel only should turn", NAMES[motor], POWERS[power]));
        for (int i = 0; i < NAMES.length; i++) {
            if (motors[i] != null) {
                telemetry.addData(NAMES[i], "%.1f power, %.0f ticks/s", motors[i].getPower(), motors[i].getVelocity());
            }
        }
        log.update();
    }

    @Override
    public void stop() {
        for (DcMotorEx m : motors) if (m != null) m.setPower(0);
        if (log != null) log.close();
    }

    private void beginStep() {
        int motor = step / (POWERS.length + 1);
        int power = step % (POWERS.length + 1);
        for (int i = 0; i < NAMES.length; i++) {
            if (motors[i] == null) continue;
            motors[i].setPower(i == motor && power < POWERS.length ? POWERS[power] : 0);
        }
        measuring = false;
        timer.reset();
    }

    private int position(int i) {
        return motors[i] == null ? 0 : motors[i].getCurrentPosition();
    }

    private void finish() {
        done = true;
        for (DcMotorEx m : motors) if (m != null) m.setPower(0);
        results.clear();
        for (int m = 0; m < NAMES.length; m++) {
            results.add(NAMES[m] + ": " + (motors[m] == null ? "not in the configuration" : verdict(m)));
        }
        log.addLine("========== RESULTS ==========");
        log.update();
        for (String line : results) {
            log.addLine(line);
            log.update();
        }
    }

    /** What the readings say about motor m. Package-private so the desktop tests can check it. */
    String verdict(int m) {
        double low = speed[m][0][m];
        double high = speed[m][1][m];

        // Whichever other encoder moved most while this motor ran alone.
        int other = -1;
        double otherSpeed = 0;
        for (int i = 0; i < NAMES.length; i++) {
            if (i == m) continue;
            double s = Math.max(Math.abs(speed[m][0][i]), Math.abs(speed[m][1][i]));
            if (s > otherSpeed) {
                otherSpeed = s;
                other = i;
            }
        }
        String counts = String.format(Locale.US, " (%.0f ticks/s at 0.2, %.0f at 0.5)", low, high);

        if (Math.abs(low) < STILL_TICKS_PER_SEC && Math.abs(high) < STILL_TICKS_PER_SEC) {
            if (otherSpeed >= STILL_TICKS_PER_SEC) {
                return "ENCODER IN THE WRONG PORT: running " + NAMES[m] + " moved the " + NAMES[other]
                        + " encoder. Swap the encoder cables so each is in its own motor's port."
                        + " Until then the hub drives these motors at full power.";
            }
            return "NO ENCODER COUNTS. If the wheel turned, the encoder cable is unplugged or broken,"
                    + " and RUN_USING_ENCODER drives this motor at FULL power whatever it is asked."
                    + " If the wheel did not turn, check the power cable and port.";
        }
        if (low < 0 && high < 0) {
            return "COUNTS BACKWARDS" + counts + ". The hub will drive it at full power; check the encoder cable.";
        }
        if (otherSpeed >= STILL_TICKS_PER_SEC) {
            return "ALSO MOVED THE " + NAMES[other] + " ENCODER" + counts
                    + ". Encoder cables may be crossed, or the wheels are touching something.";
        }
        double ratio = high / Math.max(low, 1);
        if (ratio < 1.8 || ratio > 3.5) {
            return String.format(Locale.US, "SPEED DOES NOT FOLLOW POWER%s: 0.5 power ran %.1fx as fast as 0.2,"
                    + " expected about 2.5x. Check the encoder cable.", counts, ratio);
        }
        MotorConfigurationType type = motors[m].getMotorType();
        if (type != null && type.getAchieveableMaxTicksPerSecond() > 0) {
            double expected = POWERS[1] * type.getAchieveableMaxTicksPerSecond();
            double fraction = high / expected;
            if (fraction < 0.7 || fraction > 1.3) {
                return String.format(Locale.US, "SPEED OFF%s: at 0.5 it ran %.0f%% of what a %s should."
                        + " Check that the motor type in the robot configuration matches the gearbox label.",
                        counts, fraction * 100, type.getName());
            }
        }
        return "OK" + counts;
    }

    private void showResults() {
        telemetry.addLine("DONE. Results (also in the log file):");
        for (String line : results) telemetry.addLine(line);
        telemetry.update();
    }

    private static String describeType(DcMotorEx motor) {
        MotorConfigurationType type = motor.getMotorType();
        if (type == null) return "unknown motor type";
        return String.format(Locale.US, "%s, %.0f rpm, %.1f ticks/rev", type.getName(), type.getMaxRPM(), type.getTicksPerRev());
    }
}

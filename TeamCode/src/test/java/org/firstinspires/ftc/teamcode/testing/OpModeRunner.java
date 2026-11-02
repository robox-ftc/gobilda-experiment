package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.devices.GamePadReadings;
import org.mockito.MockedConstruction;

/**
 * Runs a real OpMode on the desktop, the way the robot controller would.
 *
 * <p>The order of calls is the SDK's own, read out of
 * {@code OpMode.internalRunOpMode} in RobotCore 11.0.0: {@code init()} with no
 * telemetry update after it; then, each cycle, the latest gamepads copied in and
 * {@code init_loop()}; on START, {@code start()}; then {@code loop()} each cycle;
 * finally {@code stop()}. Every call except {@code init()} is followed by a
 * {@code telemetry.update()}. When an OpMode ends the robot controller runs its
 * default OpMode, whose {@code startSafe()} sets every motor to power 0, so this
 * does too.
 *
 * <p>Time is {@link DesktopRobot#clock}: every ElapsedTime the OpMode creates
 * reads it, so a test can run a whole autonomous in milliseconds and a window
 * can run it in real time. An {@code @Autonomous} OpMode is stopped after
 * {@link #AUTONOMOUS_SECONDS}, as the Driver Station's timer does.
 *
 * <p>An exception thrown by the OpMode stops it, as on the robot, and is kept in
 * {@link #failure()} so it can be shown rather than lost.
 *
 * <p>Construct, run and close it on one thread: the clock is Mockito's
 * mockConstruction, which is per-thread.
 */
public final class OpModeRunner implements AutoCloseable {

    public enum Phase { INIT, RUNNING, STOPPED }

    /** The FTC autonomous period. */
    public static final double AUTONOMOUS_SECONDS = 30.0;

    private final DesktopRobot robot;
    private final DriverStation ds;
    private final MockedConstruction<ElapsedTime> timers;
    private final OpMode opMode;
    private final boolean autonomous;
    private final String name;

    private Phase phase = Phase.INIT;
    private double runSeconds;
    private Throwable failure;

    public OpModeRunner(Class<? extends OpMode> type, DesktopRobot robot, DriverStation ds) {
        if (LinearOpMode.class.isAssignableFrom(type)) {
            throw new IllegalArgumentException(type.getSimpleName()
                    + " is a LinearOpMode, which runs on its own thread; only OpMode is supported");
        }
        this.robot = robot;
        this.ds = ds;
        this.autonomous = type.isAnnotationPresent(Autonomous.class);
        this.name = displayName(type);

        // Installed before construction: OpModes create timers in field initialisers.
        timers = robot.clock.install();
        try {
            opMode = type.getDeclaredConstructor().newInstance();
        } catch (ReflectiveOperationException e) {
            timers.close();
            throw new IllegalArgumentException("cannot construct " + type.getName(), e);
        }
        opMode.hardwareMap = robot.hardwareMap;
        opMode.telemetry = robot.telemetry.device;
        opMode.gamepad1 = ds.gamepad1;
        opMode.gamepad2 = ds.gamepad2;

        run(opMode::init, false);
    }

    /**
     * One robot-controller cycle: copy the gamepads in, run {@code init_loop()}
     * or {@code loop()} as the phase says, then let {@code dtSec} of robot time
     * pass and advance the modelled hardware.
     *
     * @return the merged gamepad readings for this cycle, for display
     */
    public GamePadReadings cycle(double dtSec) {
        ds.copyIn();
        GamePadReadings readings = ds.peek();
        if (phase == Phase.INIT) {
            run(opMode::init_loop, true);
        } else if (phase == Phase.RUNNING) {
            run(opMode::loop, true);
            runSeconds += dtSec;
            if (autonomous && runSeconds >= AUTONOMOUS_SECONDS - 1e-9) stop();
        }
        robot.clock.advance(dtSec);
        robot.step(dtSec);
        return readings;
    }

    /** The Driver Station's START. Does nothing unless the OpMode is in INIT. */
    public void start() {
        if (phase != Phase.INIT) return;
        ds.copyIn();
        phase = Phase.RUNNING;
        run(opMode::start, true);
    }

    /** The Driver Station's STOP, or the end of the autonomous period. */
    public void stop() {
        if (phase == Phase.STOPPED) return;
        phase = Phase.STOPPED;
        if (failure == null) {
            ds.copyIn();
            run(opMode::stop, true);
        }
        stopAllMotors();
    }

    private void stopAllMotors() {
        for (DcMotorSimple motor : robot.hardwareMap.getAll(DcMotorSimple.class)) {
            motor.setPower(0);
        }
    }

    private void run(Runnable userCode, boolean thenUpdateTelemetry) {
        if (failure != null) return;
        try {
            userCode.run();
            if (thenUpdateTelemetry) robot.telemetry.device.update();
        } catch (RuntimeException | LinkageError e) {
            failure = e;
            phase = Phase.STOPPED;
            stopAllMotors();
        }
    }

    public Phase phase() {
        return phase;
    }

    /** Robot time since START, in seconds. */
    public double runSeconds() {
        return runSeconds;
    }

    /** What the OpMode threw, or null. */
    public Throwable failure() {
        return failure;
    }

    public boolean isAutonomous() {
        return autonomous;
    }

    /** The name the Driver Station lists it under. */
    public String name() {
        return name;
    }

    public OpMode opMode() {
        return opMode;
    }

    @Override
    public void close() {
        timers.close();
    }

    static String displayName(Class<?> type) {
        Autonomous auto = type.getAnnotation(Autonomous.class);
        if (auto != null && !auto.name().isEmpty()) return auto.name();
        TeleOp teleOp = type.getAnnotation(TeleOp.class);
        if (teleOp != null && !teleOp.name().isEmpty()) return teleOp.name();
        return type.getSimpleName();
    }
}

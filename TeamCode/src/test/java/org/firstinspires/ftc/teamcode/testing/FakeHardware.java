package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.reflect.InvocationHandler;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.LongSupplier;

/**
 * Stand-ins for the hardware, for the interactive {@link GamepadHarness}.
 *
 * <p>Unlike the unit tests, which use Mockito to assert about single calls, the
 * harness needs to <em>read back</em> the current state every cycle and display
 * it. These record what was last commanded.
 *
 * <p>They are built with {@link Proxy} rather than written out longhand because
 * DcMotorEx inherits some forty methods, almost none of which this code touches.
 * Anything not handled here returns a harmless zero/null.
 */
public final class FakeHardware {

    private FakeHardware() {}

    /**
     * A first-order model of a motor that has to spin something heavy up.
     *
     * <p>A recorder can say what was commanded; it cannot say what the robot
     * would be able to read back. Where a plant is attached, {@code
     * getVelocity()} reports the modelled speed instead of the command.
     *
     * <p><b>Both numbers are guesses</b> and neither was measured on the real
     * robot, so read a shape off this and never a value. The point is that a
     * readback which is <em>not</em> the command exists at all: without one,
     * {@code Launcher.getLeftWheelRPM()} just repeats the command, and wheels
     * appear to arrive the moment they are told to go.
     */
    public static final class Plant {
        /** Shaft speed at power 1.0, in encoder ticks per second. */
        public final double freeTicksPerSec;
        /** Seconds to reach 63% of a step change while being driven. */
        public final double tauSeconds;
        /**
         * Seconds to lose 63% of the speed with the power off and the motor
         * set to FLOAT. Longer than {@link #tauSeconds}, because nothing is
         * holding the shaft: it is friction against the flywheel's inertia.
         */
        public final double coastTauSeconds;

        public Plant(double freeTicksPerSec, double tauSeconds, double coastTauSeconds) {
            this.freeTicksPerSec = freeTicksPerSec;
            this.tauSeconds = tauSeconds;
            this.coastTauSeconds = coastTauSeconds;
        }
    }

    /** A motor that remembers the last thing asked of it. */
    public static final class Motor {
        public final String name;
        public volatile double power;
        public volatile double velocity;

        /**
         * Whether the last command was a velocity rather than a power. That
         * decides what zero means: {@code setPower(0)} lets a FLOAT motor coast,
         * while {@code setVelocity(0)} keeps the motor enabled with a target of
         * zero (LynxDcMotorController, SDK 11.0.0 bytecode), so the velocity
         * loop holds it there.
         */
        public volatile boolean velocityCommanded;

        /**
         * Optional model of what the motor does with the power it was given.
         * Null everywhere by default — a recorder should not invent a number
         * nobody modelled — so {@code getVelocity()} keeps returning the
         * commanded velocity on every motor that has no plant.
         */
        public volatile Plant plant;

        /** Modelled shaft speed in encoder ticks per second. */
        public volatile double simTicksPerSec;

        public volatile int targetPosition;
        public volatile DcMotor.RunMode mode;
        public volatile DcMotor.Direction direction = DcMotor.Direction.FORWARD;

        /**
         * What the motor does at zero power. FLOAT is the SDK's default and
         * means coasting; BRAKE shorts the windings and stops it quickly. Every
         * device class sets this deliberately - Drivetrain BRAKE, and Intake,
         * Ramp and Launcher FLOAT - and it was being silently discarded here.
         */
        public volatile DcMotor.ZeroPowerBehavior zeroPowerBehavior =
                DcMotor.ZeroPowerBehavior.FLOAT;
        public final DcMotorEx device;

        /** Advances the model by {@code dtSec}. A no-op with nothing modelled. */
        public void step(double dtSec) {
            if (dtSec <= 0 || plant == null) return;

            double target = velocityCommanded
                    ? Math.max(-plant.freeTicksPerSec, Math.min(plant.freeTicksPerSec, velocity))
                    : power * plant.freeTicksPerSec;
            // A motor let go with FLOAT set is not being slowed by anything but
            // friction, so it runs down far more slowly than it spun up. That
            // takes a setPower(0): the velocity loop drives a target of zero.
            boolean coasting = !velocityCommanded && power == 0
                    && zeroPowerBehavior == DcMotor.ZeroPowerBehavior.FLOAT;
            double tau = coasting ? plant.coastTauSeconds : plant.tauSeconds;
            // Explicit Euler on dv/dt = (target - v) / tau, clamped so a long
            // frame cannot overshoot past the target and oscillate.
            simTicksPerSec += (target - simTicksPerSec) * Math.min(1.0, dtSec / tau);
        }

        Motor(String name) {
            this.name = name;
            this.device = (DcMotorEx) Proxy.newProxyInstance(
                    DcMotorEx.class.getClassLoader(),
                    new Class<?>[]{DcMotorEx.class},
                    new MotorHandler(this));
        }
    }

    public static Motor motor(String name) {
        return new Motor(name);
    }

    private static final class MotorHandler implements InvocationHandler {
        private final Motor state;

        MotorHandler(Motor state) {
            this.state = state;
        }

        @Override
        public Object invoke(Object proxy, Method method, Object[] args) {
            Object handled = handleObjectMethods(proxy, method, args, state.name);
            if (handled != NOT_HANDLED) return handled;

            switch (method.getName()) {
                case "setPower":
                    state.power = (Double) args[0];
                    state.velocityCommanded = false;
                    return null;
                case "getPower":
                    return state.power;
                case "setVelocity":
                    state.velocity = (Double) args[0];
                    state.velocityCommanded = true;
                    return null;
                case "getVelocity":
                    // With a plant attached this is deliberately NOT the command:
                    // it is what the robot code would read off the encoder.
                    return state.plant == null ? state.velocity : state.simTicksPerSec;
                case "setTargetPosition":
                    state.targetPosition = (Integer) args[0];
                    return null;
                case "getTargetPosition":
                    return state.targetPosition;
                case "getCurrentPosition":
                    // There is no position model, so the mechanism teleports.
                    // Nothing in the robot code reads it.
                    return state.targetPosition;
                case "setMode":
                    state.mode = (DcMotor.RunMode) args[0];
                    return null;
                case "getMode":
                    return state.mode;
                case "setZeroPowerBehavior":
                    state.zeroPowerBehavior = (DcMotor.ZeroPowerBehavior) args[0];
                    return null;
                case "getZeroPowerBehavior":
                    return state.zeroPowerBehavior;
                case "resetDeviceConfigurationForOpMode":
                    // Deliberately nothing. The device classes call it first and
                    // then set the direction, run mode and zero-power behaviour
                    // they want, so there is nothing here worth undoing - but it
                    // is handled explicitly rather than falling through to a
                    // silent default, which is how the last two defects hid.
                    return null;
                case "setDirection":
                    state.direction = (DcMotor.Direction) args[0];
                    return null;
                case "getDirection":
                    return state.direction;
                case "getDeviceName":
                    return state.name;
                default:
                    return defaultFor(method.getReturnType());
            }
        }
    }

    /**
     * Collects whatever the device classes report, so the harness can show the
     * same lines the drivers would see on the Driver Station — <em>including
     * how often they would see them change</em>.
     *
     * <p>A robot loop runs at about 50 Hz, but the Driver Station does not
     * refresh at 50 Hz. {@code TelemetryImpl} holds a transmission timer and
     * {@code tryUpdate} sends nothing until it passes {@code
     * msTransmissionInterval}, which the SDK constructor sets to <b>250 ms</b>
     * (read out of RobotCore 11.0.0, not remembered). So a driver sees roughly
     * four frames a second, each one whole; the loops in between are dropped.
     *
     * <p>This models that visible behaviour rather than TelemetryImpl's
     * internals: {@code update()} composes what has been added since the last
     * one and clears it, and the composed frame replaces what the viewer sees
     * only when the interval has passed. Everything the OpMode adds between
     * transmissions is simply never seen, which is the point — a number that
     * flickers 50 times a second on this screen would not be readable on a real
     * one.
     */
    public static final class TelemetryRecorder {
        /** Added since the last update(), not yet composed into a frame. */
        private final List<String> pending = Collections.synchronizedList(new ArrayList<>());

        /** The frame a driver would currently be looking at. */
        private volatile List<String> visible = Collections.emptyList();

        /** The SDK default, and settable the same way the OpMode would. */
        private volatile int msTransmissionInterval = 250;

        // Far enough in the past that the first frame goes out at once. The
        // real thing would hold it back for a quarter second, which buys
        // nothing here and would make a single-cycle test see an empty screen.
        private volatile long lastTransmitNanos = Long.MIN_VALUE / 4;

        public final Telemetry device;

        /** Where "now" comes from: the PC's clock, or a SimClock's. */
        private final LongSupplier nanoClock;

        public TelemetryRecorder() {
            this(System::nanoTime);
        }

        /** @param nanoClock robot time in nanoseconds, such as {@code SimClock::nanos} */
        public TelemetryRecorder(LongSupplier nanoClock) {
            this.nanoClock = nanoClock;
            this.device = (Telemetry) Proxy.newProxyInstance(
                    Telemetry.class.getClassLoader(),
                    new Class<?>[]{Telemetry.class},
                    (proxy, method, args) -> {
                        Object handled = handleObjectMethods(proxy, method, args, "telemetry");
                        if (handled != NOT_HANDLED) return handled;

                        switch (method.getName()) {
                            case "addData":
                                pending.add(args[0] + " " + formatValue(args));
                                return null;
                            case "addLine":
                                pending.add(args.length > 0 ? String.valueOf(args[0]) : "");
                                return null;
                            case "update":
                                return transmit();
                            case "clear":
                            case "clearAll":
                                pending.clear();
                                return null;
                            case "setMsTransmissionInterval":
                                msTransmissionInterval = (Integer) args[0];
                                return null;
                            case "getMsTransmissionInterval":
                                return msTransmissionInterval;
                            default:
                                return defaultFor(method.getReturnType());
                        }
                    });
        }

        /**
         * What the Driver Station would be showing right now. Unchanged between
         * transmissions, so reading it every loop is cheap and honest.
         */
        public List<String> visible() {
            return visible;
        }

        /** How long a frame stays on screen. Set to 0 to transmit every update. */
        public void setMsTransmissionInterval(int ms) {
            msTransmissionInterval = ms;
        }

        /**
         * One {@code telemetry.update()}: compose the frame and clear, then put
         * it on screen if the transmission interval has passed.
         *
         * @return whether it went out, which is what the SDK's update() returns.
         */
        private boolean transmit() {
            List<String> composed;
            synchronized (pending) {
                composed = new ArrayList<>(pending);
                pending.clear();
            }
            long now = nanoClock.getAsLong();
            if (now - lastTransmitNanos < msTransmissionInterval * 1_000_000L) {
                return false;
            }
            lastTransmitNanos = now;
            visible = composed;
            return true;
        }

        private static String formatValue(Object[] args) {
            if (args.length == 2) return String.valueOf(args[1]);
            if (args.length == 3 && args[1] instanceof String) {
                // addData(caption, format, args...) — the varargs arrive boxed.
                Object tail = args[2];
                Object[] values = tail instanceof Object[] ? (Object[]) tail : new Object[]{tail};
                try {
                    return String.format((String) args[1], values);
                } catch (RuntimeException e) {
                    return String.valueOf(args[1]);
                }
            }
            return "";
        }
    }

    // ------------------------------------------------------------------ plumbing

    private static final Object NOT_HANDLED = new Object();

    /** Proxies receive toString/hashCode/equals too, and must answer sensibly. */
    private static Object handleObjectMethods(Object proxy, Method method, Object[] args, String name) {
        switch (method.getName()) {
            case "toString":
                return method.getParameterCount() == 0 ? name : NOT_HANDLED;
            case "hashCode":
                return method.getParameterCount() == 0 ? System.identityHashCode(proxy) : NOT_HANDLED;
            case "equals":
                return method.getParameterCount() == 1 ? proxy == args[0] : NOT_HANDLED;
            default:
                return NOT_HANDLED;
        }
    }

    private static Object defaultFor(Class<?> type) {
        if (!type.isPrimitive()) return null;
        if (type == boolean.class) return false;
        if (type == double.class) return 0d;
        if (type == float.class) return 0f;
        if (type == long.class) return 0L;
        if (type == int.class) return 0;
        if (type == short.class) return (short) 0;
        if (type == byte.class) return (byte) 0;
        if (type == char.class) return (char) 0;
        return null; // void
    }
}

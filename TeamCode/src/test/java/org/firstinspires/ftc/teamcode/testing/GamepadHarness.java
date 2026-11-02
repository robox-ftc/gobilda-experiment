package org.firstinspires.ftc.teamcode.testing;

import static org.lwjgl.glfw.GLFW.GLFW_GAMEPAD_AXIS_LEFT_TRIGGER;
import static org.lwjgl.glfw.GLFW.GLFW_GAMEPAD_AXIS_LEFT_X;
import static org.lwjgl.glfw.GLFW.GLFW_GAMEPAD_AXIS_LEFT_Y;
import static org.lwjgl.glfw.GLFW.GLFW_GAMEPAD_AXIS_RIGHT_TRIGGER;
import static org.lwjgl.glfw.GLFW.GLFW_GAMEPAD_AXIS_RIGHT_X;
import static org.lwjgl.glfw.GLFW.GLFW_GAMEPAD_AXIS_RIGHT_Y;
import static org.lwjgl.glfw.GLFW.GLFW_GAMEPAD_BUTTON_A;
import static org.lwjgl.glfw.GLFW.GLFW_GAMEPAD_BUTTON_B;
import static org.lwjgl.glfw.GLFW.GLFW_GAMEPAD_BUTTON_BACK;
import static org.lwjgl.glfw.GLFW.GLFW_GAMEPAD_BUTTON_DPAD_DOWN;
import static org.lwjgl.glfw.GLFW.GLFW_GAMEPAD_BUTTON_DPAD_LEFT;
import static org.lwjgl.glfw.GLFW.GLFW_GAMEPAD_BUTTON_DPAD_RIGHT;
import static org.lwjgl.glfw.GLFW.GLFW_GAMEPAD_BUTTON_DPAD_UP;
import static org.lwjgl.glfw.GLFW.GLFW_GAMEPAD_BUTTON_GUIDE;
import static org.lwjgl.glfw.GLFW.GLFW_GAMEPAD_BUTTON_LAST;
import static org.lwjgl.glfw.GLFW.GLFW_GAMEPAD_BUTTON_LEFT_BUMPER;
import static org.lwjgl.glfw.GLFW.GLFW_GAMEPAD_BUTTON_LEFT_THUMB;
import static org.lwjgl.glfw.GLFW.GLFW_GAMEPAD_BUTTON_RIGHT_BUMPER;
import static org.lwjgl.glfw.GLFW.GLFW_GAMEPAD_BUTTON_RIGHT_THUMB;
import static org.lwjgl.glfw.GLFW.GLFW_GAMEPAD_BUTTON_START;
import static org.lwjgl.glfw.GLFW.GLFW_GAMEPAD_BUTTON_X;
import static org.lwjgl.glfw.GLFW.GLFW_GAMEPAD_BUTTON_Y;
import static org.lwjgl.glfw.GLFW.GLFW_JOYSTICK_1;
import static org.lwjgl.glfw.GLFW.GLFW_JOYSTICK_LAST;
import static org.lwjgl.glfw.GLFW.GLFW_PRESS;
import static org.lwjgl.glfw.GLFW.glfwGetGamepadName;
import static org.lwjgl.glfw.GLFW.glfwGetGamepadState;
import static org.lwjgl.glfw.GLFW.glfwGetJoystickName;
import static org.lwjgl.glfw.GLFW.glfwInit;
import static org.lwjgl.glfw.GLFW.glfwJoystickIsGamepad;
import static org.lwjgl.glfw.GLFW.glfwJoystickPresent;
import static org.lwjgl.glfw.GLFW.glfwPollEvents;
import static org.lwjgl.glfw.GLFW.glfwTerminate;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.devices.GamePadReadings;
import org.lwjgl.glfw.GLFWGamepadState;

import java.util.Locale;

/**
 * Runs a real OpMode from a real gamepad plugged into this PC.
 *
 * <p>The OpMode is the one in TeamCode, unmodified: {@link OpModeRunner} gives it
 * a {@link DesktopHardwareMap} of stand-in motors and calls it the way the robot
 * controller does. The window or console opens in INIT, so the pad can make the
 * OpMode's own init_loop choices (side, mode); START on the pad, or S in the
 * window, runs it. An autonomous stops itself after 30 seconds.
 *
 * <p>{@code gradlew :TeamCode:gamepadGui} for the field view,
 * {@code gradlew :TeamCode:gamepadHarness} for a console dashboard. Add
 * {@code --args="--opmode=SomeOpMode"} to choose another OpMode by class name;
 * the default is {@value #DEFAULT_OPMODE}, the one OpMode this branch registers.
 * Not from the gutter next to {@link #main}: that route asks Gradle for a Java
 * source set an Android module does not have. See src/test/README.md.
 *
 * <p>What it will NOT tell you: whether the robot physically does the right
 * thing. The field view is dead reckoning off the recorded motor powers.
 */
public final class GamepadHarness {

    /** The OpMode run when no {@code --opmode=} is given. */
    static final String DEFAULT_OPMODE = "BIOBUZZAuto";

    /** The Driver Station applies a deadzone before sending; without one, sticks drift. */
    private static final double STICK_DEADZONE = 0.05;

    private static final long CYCLE_MS = 20; // 50 Hz, about the robot's loop rate

    public static void main(String[] args) {
        boolean probeOnly = false;
        boolean rawOnly = false;
        boolean gui = false;
        String opModeName = DEFAULT_OPMODE;
        for (String arg : args) {
            if (arg.equals("--probe")) probeOnly = true;
            else if (arg.equals("--raw")) rawOnly = true;
            else if (arg.equals("--gui")) gui = true;
            else if (arg.startsWith("--opmode=")) opModeName = arg.substring("--opmode=".length());
        }

        Class<? extends OpMode> type = null;
        if (!probeOnly && !rawOnly) {
            try {
                type = findOpMode(opModeName);
            } catch (IllegalArgumentException e) {
                System.err.println(e.getMessage());
                System.exit(1);
            }
        }

        if (!glfwInit()) {
            System.err.println("Could not start GLFW, so no gamepad can be read.");
            System.exit(1);
        }
        try {
            int[] pads = findGamepads();
            printDetected(pads);
            if (probeOnly) return;
            if (pads.length == 0 && !gui) {
                System.out.println("\nPlug in or pair a controller and run this again.");
                return;
            }
            if (rawOnly) {
                runRaw(pads[0]);
            } else if (gui) {
                runGui(pads, type);
            } else {
                run(pads, type);
            }
        } finally {
            glfwTerminate();
        }
    }

    /** A TeamCode OpMode by simple class name, or any OpMode by full name. */
    static Class<? extends OpMode> findOpMode(String name) {
        String className = name.contains(".") ? name : "org.firstinspires.ftc.teamcode." + name;
        Class<?> found;
        try {
            found = Class.forName(className);
        } catch (ClassNotFoundException e) {
            throw new IllegalArgumentException("No OpMode class " + className
                    + ". Pass --opmode= with the class name, not the Driver Station name.");
        }
        if (!OpMode.class.isAssignableFrom(found)) {
            throw new IllegalArgumentException(className + " is not an OpMode.");
        }
        return found.asSubclass(OpMode.class);
    }

    // --------------------------------------------------------------- discovery

    private static int[] findGamepads() {
        glfwPollEvents();
        int[] found = new int[GLFW_JOYSTICK_LAST + 1];
        int count = 0;
        for (int jid = GLFW_JOYSTICK_1; jid <= GLFW_JOYSTICK_LAST; jid++) {
            if (glfwJoystickPresent(jid) && glfwJoystickIsGamepad(jid)) {
                found[count++] = jid;
            }
        }
        int[] result = new int[count];
        System.arraycopy(found, 0, result, 0, count);
        return result;
    }

    private static void printDetected(int[] pads) {
        System.out.println("Controllers found: " + pads.length);
        for (int i = 0; i < pads.length; i++) {
            System.out.printf("  gamepad%d  %s%n", i + 1, describe(pads[i]));
        }
        for (int jid = GLFW_JOYSTICK_1; jid <= GLFW_JOYSTICK_LAST; jid++) {
            if (glfwJoystickPresent(jid) && !glfwJoystickIsGamepad(jid)) {
                System.out.printf("  (ignored, no gamepad mapping) %s%n", glfwGetJoystickName(jid));
            }
        }
    }

    private static String describe(int jid) {
        String name = glfwGetGamepadName(jid);
        return name != null ? name : String.valueOf(glfwGetJoystickName(jid));
    }

    // ------------------------------------------------------------------ running

    /** Reads the pads into the Driver Station; false if pad 1 stopped reporting. */
    private static boolean poll(int[] pads, DriverStation ds, GLFWGamepadState state) {
        boolean ok = true;
        if (pads.length > 0) {
            if (glfwGetGamepadState(pads[0], state)) {
                applyTo(ds.held1, state);
            } else {
                ok = false;
            }
        }
        if (pads.length > 1 && glfwGetGamepadState(pads[1], state)) {
            applyTo(ds.held2, state);
        }
        return ok;
    }

    private static void run(int[] pads, Class<? extends OpMode> type) {
        DriverStation ds = new DriverStation();
        DesktopRobot robot = new DesktopRobot();
        OpModeRunner runner = new OpModeRunner(type, robot, ds);

        System.out.println();
        System.out.println("Running " + runner.name() + " on stand-in motors. It is in INIT:");
        System.out.println("make its choices on the pad, then press START to run it.");
        System.out.println("START again after it stops goes back to INIT. BACK + START quits.");
        System.out.println();

        GLFWGamepadState state = GLFWGamepadState.create();
        String lastFrame = "";
        boolean warnedNotReporting = false;
        boolean startWasDown = false;
        long lastNanos = System.nanoTime();

        try {
            while (true) {
                glfwPollEvents();
                if (!poll(pads, ds, state) && !warnedNotReporting) {
                    System.out.println("!! glfwGetGamepadState is returning false: the pad is not being");
                    System.out.println("!! read. Anything below this line is stale. Check that the");
                    System.out.println("!! controller is awake, then run with --args=\"--raw\".");
                    warnedNotReporting = true;
                }
                if (ds.held1.back && ds.held1.start) {
                    System.out.println("\nStopped.");
                    return;
                }
                boolean startPressed = ds.held1.start && !startWasDown && !ds.held1.back;
                startWasDown = ds.held1.start;
                if (startPressed) {
                    if (runner.phase() == OpModeRunner.Phase.STOPPED) {
                        runner.close();
                        robot = new DesktopRobot();
                        runner = new OpModeRunner(type, robot, ds);
                    } else {
                        runner.start();
                    }
                }

                long now = System.nanoTime();
                double dt = Math.min(0.1, (now - lastNanos) / 1e9);
                lastNanos = now;
                GamePadReadings readings = runner.cycle(dt);

                String frame = render(readings, runner, robot);
                if (!frame.equals(lastFrame)) {
                    System.out.println(frame);
                    lastFrame = frame;
                }
                sleep();
            }
        } finally {
            runner.close();
        }
    }

    // ----------------------------------------------------------------- gui mode

    /**
     * The same as {@link #run}, drawn instead of printed. The OpMode keeps
     * running on this thread; {@link RobotView} is handed a finished snapshot
     * each cycle, so no robot class is touched from Swing's event thread.
     *
     * <p>Falls back to the console if there is no display.
     */
    private static void runGui(int[] pads, Class<? extends OpMode> type) {
        RobotView view = new RobotView();
        if (!view.open()) {
            run(pads, type);
            return;
        }

        DriverStation ds = new DriverStation();
        DesktopRobot robot = new DesktopRobot();
        OpModeRunner runner = new OpModeRunner(type, robot, ds);
        RobotSim sim = new RobotSim();
        BallPath ball = new BallPath();

        System.out.println();
        System.out.println("Window open, running " + runner.name() + " on stand-in motors.");
        System.out.println("It is in INIT: make its choices on the pad, then START (or S) runs it.");
        System.out.println("R restarts. BACK + START on the pad, or closing the window, quits.");

        if (pads.length == 0) {
            view.setInputStatus("NO CONTROLLER FOUND - S still runs it with no choices made");
            System.out.println("No controller found. The window opens anyway.");
        }

        GLFWGamepadState state = GLFWGamepadState.create();
        boolean warnedNotReporting = false;
        boolean startWasDown = false;
        long lastNanos = System.nanoTime();

        try {
            while (view.isOpen()) {
                glfwPollEvents();
                if (!poll(pads, ds, state) && !warnedNotReporting) {
                    System.out.println("!! glfwGetGamepadState is returning false: the pad is not being");
                    System.out.println("!! read, so the window is stale. Try --args=\"--raw\".");
                    warnedNotReporting = true;
                }
                if (pads.length > 0 && ds.held1.back && ds.held1.start) break;

                boolean padStart = ds.held1.start && !startWasDown && !ds.held1.back;
                startWasDown = ds.held1.start;
                boolean restart = view.consumeResetRequest();
                boolean startPressed = view.consumeStartRequest() || padStart;
                if (startPressed && runner.phase() == OpModeRunner.Phase.STOPPED) restart = true;

                if (restart) {
                    runner.close();
                    robot = new DesktopRobot();
                    runner = new OpModeRunner(type, robot, ds);
                    sim.reset();
                    ball.reset();
                    view.clearTrail();
                } else if (startPressed) {
                    runner.start();
                }

                long now = System.nanoTime();
                double dt = (now - lastNanos) / 1e9;
                lastNanos = now;
                // A hitch - a breakpoint, a dragged window, a GC pause - must not
                // fling the robot across the field on the next frame.
                if (dt > 0.1) dt = 0.1;

                GamePadReadings readings = runner.cycle(dt);
                ball.step(robot.intakeFlow(), robot.rampFlow(), robot.flywheelSpeed(), dt);
                sim.step(robot.frontLeft.power, robot.frontRight.power,
                        robot.rearLeft.power, robot.rearRight.power, dt);

                view.update(readings, robot, sim, ball, robot.telemetry.visible(), runner);
                sleep();
            }
        } finally {
            runner.close();
            view.close();
        }
        System.out.println("\nStopped.");
    }

    // ----------------------------------------------------------------- raw mode

    /**
     * Prints what GLFW reports, with no robot code in the way. Use it when the
     * dashboard looks stuck: it separates "the pad is not being read" from "the
     * pad is read fine and the command really is zero".
     *
     * <p>The numbers are GLFW's own, before any conversion. Sticks run -1..1.
     * Triggers also run -1..1, where <b>-1.000 is released</b> and +1.000 is
     * pulled all the way - that is the resting value, not a failure.
     */
    private static void runRaw(int jid) {
        System.out.println();
        System.out.println("Raw GLFW state for " + describe(jid) + ".");
        System.out.println("Sticks -1..1. Triggers -1 released to +1 pulled.");
        System.out.println("Hold BACK + START together to quit.");
        System.out.println();

        GLFWGamepadState state = GLFWGamepadState.create();
        String last = "";
        long lastPrint = 0;

        while (true) {
            glfwPollEvents();

            if (!glfwGetGamepadState(jid, state)) {
                System.out.println("glfwGetGamepadState returned false - the pad stopped reporting.");
                return;
            }

            if (pressed(state, GLFW_GAMEPAD_BUTTON_BACK) && pressed(state, GLFW_GAMEPAD_BUTTON_START)) {
                System.out.println("\nStopped.");
                return;
            }

            StringBuilder sb = new StringBuilder();
            sb.append(String.format(Locale.ROOT,
                    "LX %+.3f  LY %+.3f  RX %+.3f  RY %+.3f  LT %+.3f  RT %+.3f  buttons ",
                    state.axes(GLFW_GAMEPAD_AXIS_LEFT_X), state.axes(GLFW_GAMEPAD_AXIS_LEFT_Y),
                    state.axes(GLFW_GAMEPAD_AXIS_RIGHT_X), state.axes(GLFW_GAMEPAD_AXIS_RIGHT_Y),
                    state.axes(GLFW_GAMEPAD_AXIS_LEFT_TRIGGER),
                    state.axes(GLFW_GAMEPAD_AXIS_RIGHT_TRIGGER)));
            int before = sb.length();
            for (int button = 0; button <= GLFW_GAMEPAD_BUTTON_LAST; button++) {
                if (pressed(state, button)) sb.append(button).append(' ');
            }
            if (sb.length() == before) sb.append('-');

            // Printed on change, but no faster than 5 Hz: raw sticks have no
            // deadzone, so a resting thumbstick jitters in the third decimal.
            String line = sb.toString();
            long now = System.currentTimeMillis();
            if (!line.equals(last) && now - lastPrint >= 200) {
                System.out.println(line);
                last = line;
                lastPrint = now;
            }

            sleep();
        }
    }

    private static void sleep() {
        try {
            Thread.sleep(CYCLE_MS);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    // -------------------------------------------------------------- translation

    /**
     * Copies one GLFW gamepad onto the SDK Gamepad the OpMode code expects.
     *
     * <p>Two conversions matter. GLFW reports triggers over -1..1 while the FTC
     * SDK uses 0..1. Stick Y is negative-up in both, so it passes straight
     * through.
     */
    private static void applyTo(Gamepad pad, GLFWGamepadState state) {
        pad.a = pressed(state, GLFW_GAMEPAD_BUTTON_A);
        pad.b = pressed(state, GLFW_GAMEPAD_BUTTON_B);
        pad.x = pressed(state, GLFW_GAMEPAD_BUTTON_X);
        pad.y = pressed(state, GLFW_GAMEPAD_BUTTON_Y);
        pad.left_bumper = pressed(state, GLFW_GAMEPAD_BUTTON_LEFT_BUMPER);
        pad.right_bumper = pressed(state, GLFW_GAMEPAD_BUTTON_RIGHT_BUMPER);
        pad.back = pressed(state, GLFW_GAMEPAD_BUTTON_BACK);
        pad.start = pressed(state, GLFW_GAMEPAD_BUTTON_START);
        pad.guide = pressed(state, GLFW_GAMEPAD_BUTTON_GUIDE);
        pad.left_stick_button = pressed(state, GLFW_GAMEPAD_BUTTON_LEFT_THUMB);
        pad.right_stick_button = pressed(state, GLFW_GAMEPAD_BUTTON_RIGHT_THUMB);

        pad.dpad_up = pressed(state, GLFW_GAMEPAD_BUTTON_DPAD_UP);
        pad.dpad_down = pressed(state, GLFW_GAMEPAD_BUTTON_DPAD_DOWN);
        pad.dpad_left = pressed(state, GLFW_GAMEPAD_BUTTON_DPAD_LEFT);
        pad.dpad_right = pressed(state, GLFW_GAMEPAD_BUTTON_DPAD_RIGHT);

        pad.left_stick_x = deadzoned(state.axes(GLFW_GAMEPAD_AXIS_LEFT_X));
        pad.left_stick_y = deadzoned(state.axes(GLFW_GAMEPAD_AXIS_LEFT_Y));
        pad.right_stick_x = deadzoned(state.axes(GLFW_GAMEPAD_AXIS_RIGHT_X));
        pad.right_stick_y = deadzoned(state.axes(GLFW_GAMEPAD_AXIS_RIGHT_Y));

        pad.left_trigger = trigger(state.axes(GLFW_GAMEPAD_AXIS_LEFT_TRIGGER));
        pad.right_trigger = trigger(state.axes(GLFW_GAMEPAD_AXIS_RIGHT_TRIGGER));
    }

    private static boolean pressed(GLFWGamepadState state, int button) {
        return state.buttons(button) == GLFW_PRESS;
    }

    private static float deadzoned(float value) {
        return Math.abs(value) < STICK_DEADZONE ? 0f : value;
    }

    private static float trigger(float glfwValue) {
        return (glfwValue + 1f) / 2f;
    }

    // ----------------------------------------------------------------- display

    /** The phase in words, as the status line of the console or window. */
    static String phaseLine(OpModeRunner runner) {
        if (runner.failure() != null) {
            Throwable f = runner.failure();
            // Package names make the message too long to read on one line, and
            // the line in the OpMode is the thing to go and look at.
            String message = f.getMessage() == null ? ""
                    : f.getMessage().replaceAll("\\b(?:[a-z][\\w$]*\\.)+(?=[A-Za-z])", "");
            String where = "";
            for (StackTraceElement frame : f.getStackTrace()) {
                if (frame.getClassName().equals(runner.opMode().getClass().getName())) {
                    where = " (" + frame.getFileName() + ":" + frame.getLineNumber() + ")";
                    break;
                }
            }
            return String.format(Locale.ROOT, "%s  CRASHED at %.2f s: %s - %s%s", runner.name(),
                    runner.runSeconds(), f.getClass().getSimpleName(), message, where);
        }
        switch (runner.phase()) {
            case INIT:
                return runner.name() + "  INIT - make its choices on the pad, then START";
            case RUNNING:
                return String.format(Locale.ROOT, "%s  RUNNING %.1f s%s", runner.name(),
                        runner.runSeconds(),
                        runner.isAutonomous() ? " of " + (int) OpModeRunner.AUTONOMOUS_SECONDS : "");
            default:
                return String.format(Locale.ROOT, "%s  STOPPED after %.1f s", runner.name(),
                        runner.runSeconds());
        }
    }

    private static String render(GamePadReadings r, OpModeRunner runner, DesktopRobot robot) {
        StringBuilder sb = new StringBuilder();
        sb.append("--------------------------------------------------------------\n");
        sb.append(phaseLine(runner)).append('\n');
        sb.append("buttons ").append(buttonsOf(r)).append('\n');
        sb.append(String.format(Locale.ROOT,
                "drive   FL %+.2f  FR %+.2f  RL %+.2f  RR %+.2f%n",
                robot.frontLeft.power, robot.frontRight.power,
                robot.rearLeft.power, robot.rearRight.power));
        sb.append(String.format(Locale.ROOT,
                "shooter %+.2f  ramp %+.2f  intake %+.2f%n",
                robot.flywheelCommand(), robot.rampFlow(), robot.intakeFlow()));
        for (String line : robot.telemetry.visible()) {
            sb.append("   | ").append(line).append('\n');
        }
        return sb.toString();
    }

    private static String buttonsOf(GamePadReadings r) {
        StringBuilder sb = new StringBuilder();
        if (r.aButton) sb.append("A ");
        if (r.bButton) sb.append("B ");
        if (r.xButton) sb.append("X ");
        if (r.yButton) sb.append("Y ");
        if (r.leftBumper) sb.append("LB ");
        if (r.rightBumper) sb.append("RB ");
        if (r.dPadUp) sb.append("Dup ");
        if (r.dPadDown) sb.append("Ddn ");
        if (r.dPadLeft) sb.append("Dlf ");
        if (r.dPadRight) sb.append("Drt ");
        return sb.length() == 0 ? "-" : sb.toString().trim();
    }
}

package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.devices.GamePadReadings;

/**
 * Drives {@link Gamepad} the same way the real robot does, so that tests can
 * exercise the "was pressed" / "was released" edges.
 *
 * <p>This class exists because of a trap in the SDK: setting a field directly,
 * as in {@code gamepad1.a = true}, does <em>not</em> make {@code aWasPressed()}
 * return true. Edge detection only runs inside {@code Gamepad.fromByteArray()},
 * which is reached from {@code Gamepad.copy()} — the call the OpMode loop makes
 * once per cycle with the state the Driver Station sent. A test that pokes the
 * fields therefore sees every edge flag stuck at false and silently passes.
 *
 * <p>So there are two gamepads per driver. {@link #held1} and {@link #held2} are
 * what the human is physically holding down; mutate those. {@link #gamepad1} and
 * {@link #gamepad2} are what the OpMode sees, and are refreshed by {@link #tick()}.
 *
 * <pre>
 *     DriverStation ds = new DriverStation();
 *     ds.held1.a = true;
 *     GamePadReadings r = ds.tick();   // r.aWasPressed == true
 *     r = ds.tick();                   // held, so r.aWasPressed == false
 * </pre>
 */
public class DriverStation {

    /** Buttons the first driver is physically holding. Mutate these. */
    public final Gamepad held1 = new Gamepad();
    /** Buttons the second driver is physically holding. Mutate these. */
    public final Gamepad held2 = new Gamepad();

    /** What the OpMode sees as {@code gamepad1}. Refreshed by {@link #tick()}. */
    public final Gamepad gamepad1 = new Gamepad();
    /** What the OpMode sees as {@code gamepad2}. Refreshed by {@link #tick()}. */
    public final Gamepad gamepad2 = new Gamepad();

    private final GamePadReadings readings = new GamePadReadings();

    /**
     * Advances one OpMode cycle: pushes the held state through the same
     * serialization path the robot uses, then rebuilds the merged readings.
     *
     * @return the readings, which are the same instance every tick (as in the
     *         OpMode, where a single GamePadReadings is reused).
     */
    public GamePadReadings tick() {
        gamepad1.copy(held1);
        gamepad2.copy(held2);
        readings.update(gamepad1, gamepad2);
        return readings;
    }

    /** Lets go of everything on both gamepads, without ticking. */
    /**
     * Copies the held state in, as the robot controller does before each call
     * into an OpMode, and nothing else. Use this when an OpMode is reading the
     * pads: {@link #tick()} also builds GamePadReadings, whose update() calls
     * {@code aWasPressed()} and friends - and in SDK 11 those calls consume the
     * edge, so the OpMode would never see the press.
     */
    public void copyIn() {
        gamepad1.copy(held1);
        gamepad2.copy(held2);
    }

    /**
     * The merged readings for display, built from copies of what the OpMode
     * sees, so that taking them leaves the OpMode's edges untouched. The edge
     * flags in the result mean nothing; the levels are what is shown.
     */
    public GamePadReadings peek() {
        Gamepad one = new Gamepad();
        Gamepad two = new Gamepad();
        one.copy(gamepad1);
        two.copy(gamepad2);
        GamePadReadings shown = new GamePadReadings();
        shown.update(one, two);
        return shown;
    }

    public void releaseAll() {
        held1.copy(new Gamepad());
        held2.copy(new Gamepad());
    }
}

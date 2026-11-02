package org.firstinspires.ftc.teamcode.devices;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.testing.DriverStation;
import org.junit.Test;

/**
 * GamePadReadings merges two physical gamepads into one set of readings, so
 * either driver can do anything. These tests pin down how that merge behaves,
 * including the edge ("was pressed") flags that the alliance selection in
 * NewTeleop.init_loop depends on.
 */
public class GamePadReadingsTest {

    private final DriverStation ds = new DriverStation();

    // ---------------------------------------------------------------- buttons

    @Test
    public void eitherDriverCanPressAnyButton() {
        ds.held2.y = true;
        GamePadReadings r = ds.tick();

        assertTrue("gamepad2 must be able to drive the launcher too", r.yButton);
    }

    @Test
    public void buttonsAreOredNotOverwritten() {
        ds.held1.b = true;
        ds.held2.b = false;
        assertTrue(ds.tick().bButton);

        ds.held1.b = false;
        ds.held2.b = true;
        assertTrue(ds.tick().bButton);

        ds.held2.b = false;
        assertFalse(ds.tick().bButton);
    }

    @Test
    public void bumpersMergeAcrossGamepads() {
        ds.held2.right_bumper = true;
        GamePadReadings r = ds.tick();

        assertTrue(r.rightBumper);
        assertFalse(r.leftBumper);
    }

    // ----------------------------------------------------------------- sticks

    @Test
    public void theStickPushedFurthestWins() {
        ds.held1.left_stick_x = 0.2f;
        ds.held2.left_stick_x = -0.9f;

        GamePadReadings r = ds.tick();

        assertEquals("larger magnitude wins, sign and all", -0.9, r.leftStickX, 1e-4);
    }

    @Test
    public void tiesGoToTheFirstGamepad() {
        ds.held1.right_stick_x = 0.5f;
        ds.held2.right_stick_x = -0.5f;

        assertEquals(0.5, ds.tick().rightStickX, 1e-4);
    }

    @Test
    public void anIdleSecondGamepadDoesNotCancelTheFirst() {
        // The merge is by magnitude, so a driver resting at 0 must never win.
        ds.held1.left_stick_y = -1.0f;

        assertEquals(-1.0, ds.tick().leftStickY, 1e-4);
    }

    // --------------------------------------------------------------- triggers

    @Test
    public void triggersTakeTheLarger() {
        ds.held1.left_trigger = 0.3f;
        ds.held2.left_trigger = 0.8f;

        assertEquals(0.8, ds.tick().leftTrigger, 1e-4);
    }

    // ------------------------------------------------------------------- dpad

    @Test
    public void dPadIsTrueWhenAnyDirectionIsHeld() {
        GamePadReadings r = ds.tick();
        assertFalse(r.dPad);

        ds.held1.dpad_left = true;
        r = ds.tick();
        assertTrue(r.dPad);
        assertTrue(r.dPadLeft);
        assertFalse(r.dPadRight);
    }

    // ------------------------------------------------------------------ edges

    @Test
    public void edgeFlagsFireOnceOnPressAndOnceOnRelease() {
        ds.held1.x = true;
        GamePadReadings r = ds.tick();
        assertTrue(r.xWasPressed);
        assertFalse(r.xWasReleased);

        r = ds.tick();
        assertFalse("still held is not a new press", r.xWasPressed);

        ds.held1.x = false;
        r = ds.tick();
        assertTrue(r.xWasReleased);
        assertFalse(r.xWasPressed);
    }

    @Test
    public void edgesFromEitherGamepadCount() {
        // Either driver may press anything, edges included.
        ds.held2.back = true;

        assertTrue(ds.tick().backWasPressed);
    }

    @Test
    public void allianceSelectionEdgesAreDistinct() {
        // NewTeleop.init_loop: X = blue, B = red, Y = far, A = near.
        ds.held1.x = true;
        GamePadReadings r = ds.tick();
        assertTrue(r.xWasPressed);
        assertFalse(r.bWasPressed);
        assertFalse(r.yWasPressed);
        assertFalse(r.aWasPressed);
    }

    @Test
    public void readingsObjectIsReusedAcrossTicks() {
        // NewTeleop keeps one GamePadReadings and calls update() on it each loop,
        // so stale values must be overwritten rather than accumulated.
        ds.held1.a = true;
        GamePadReadings first = ds.tick();

        ds.held1.a = false;
        GamePadReadings second = ds.tick();

        assertEquals(first, second);
        assertFalse("previous tick's press must not linger", second.aButton);
    }
}

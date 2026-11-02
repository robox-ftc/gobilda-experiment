package org.firstinspires.ftc.teamcode.devices;

import static org.junit.Assert.assertEquals;

import org.firstinspires.ftc.teamcode.testing.DriverStation;
import org.junit.Test;

/**
 * The launcher button map on this branch: the left trigger is the wheel power,
 * B reverses, and either bumper runs the ramp. Nothing on this branch calls it
 * live - its teleop is commented out - but it is what a teleop would use.
 */
public class LauncherControlsTest {

    private static final double EPS = 1e-6;

    private final DriverStation ds = new DriverStation();

    private LauncherControls read() {
        return LauncherControls.readControls(ds.tick());
    }

    @Test
    public void theLeftTriggerIsTheWheelPower() {
        ds.held1.left_trigger = 0.4f;

        assertEquals(0.4, read().wheelPower, EPS);
    }

    @Test
    public void bReversesTheWheelsWhateverTheTrigger() {
        ds.held1.left_trigger = 1.0f;
        ds.held1.b = true;

        assertEquals(-0.75, read().wheelPower, EPS);
    }

    @Test
    public void eitherBumperRunsTheRamp() {
        ds.held1.left_bumper = true;
        assertEquals(1.0, read().rampPower, EPS);

        ds.releaseAll();
        ds.held1.right_bumper = true;
        assertEquals(1.0, read().rampPower, EPS);
    }

    @Test
    public void bRunsTheRampBackwardsEvenWithABumperHeld() {
        ds.held1.right_bumper = true;
        ds.held1.b = true;

        assertEquals(-1.0, read().rampPower, EPS);
    }

    @Test
    public void theOldPresetButtonsAreGone() {
        ds.held1.a = true;
        ds.held1.y = true;

        assertEquals(0.0, read().wheelPower, EPS);
    }

    @Test
    public void restingControlsDoNothing() {
        LauncherControls c = read();

        assertEquals(0.0, c.wheelPower, EPS);
        assertEquals(0.0, c.rampPower, EPS);
    }
}

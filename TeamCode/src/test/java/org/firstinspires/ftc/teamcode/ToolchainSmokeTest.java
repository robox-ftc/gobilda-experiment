package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.devices.GamePadReadings;
import org.firstinspires.ftc.teamcode.testing.DriverStation;
import org.junit.Test;

/**
 * Proves the desktop test toolchain works at all: JUnit runs, Mockito can stand
 * in for an SDK hardware interface, and the real SDK Gamepad edge detection
 * functions off-robot. If this fails, nothing else in src/test will be meaningful.
 */
public class ToolchainSmokeTest {

    @Test
    public void mockitoCanStandInForAnSdkMotor() {
        DcMotorEx motor = mock(DcMotorEx.class);
        motor.setPower(0.5);
        verify(motor).setPower(0.5);
    }

    @Test
    public void gamepadEdgeDetectionWorksOffRobot() {
        DriverStation ds = new DriverStation();

        ds.held1.a = true;
        GamePadReadings r = ds.tick();
        assertTrue("rising edge should be seen on the first tick", r.aWasPressed);
        assertTrue(r.aButton);

        r = ds.tick();
        assertFalse("a held down is not a new press", r.aWasPressed);
        assertTrue(r.aButton);

        ds.held1.a = false;
        r = ds.tick();
        assertTrue(r.aWasReleased);
        assertFalse(r.aButton);
    }

    @Test
    public void analogAxesSurviveTheCopy() {
        DriverStation ds = new DriverStation();
        ds.held1.left_trigger = 0.6f;
        ds.held1.left_stick_x = -0.25f;

        GamePadReadings r = ds.tick();

        assertEquals(0.6, r.leftTrigger, 1e-4);
        assertEquals(-0.25, r.leftStickX, 1e-4);
    }
}

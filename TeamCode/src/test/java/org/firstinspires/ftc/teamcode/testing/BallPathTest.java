package org.firstinspires.ftc.teamcode.testing;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/**
 * The one-ball model the field view draws: intake, then ramp, then flywheels.
 *
 * <p>The rates are guesses, so nothing here asserts a time - only the order of
 * events.
 */
public class BallPathTest {

    private static final double DT = 0.02;

    private final BallPath ball = new BallPath();

    private void steps(int n, double intake, double ramp, double flywheels) {
        for (int i = 0; i < n; i++) ball.step(intake, ramp, flywheels, DT);
    }

    @Test
    public void aRampTurningOnAnEmptyPathIsNotAShot() {
        ball.step(0, 1.0, 1.0, 1.0);

        assertFalse(ball.takeShot());
        assertFalse(ball.hasBall);
    }

    @Test
    public void theIntakePicksOneUpAndCarriesIt() {
        steps(1, 1, 0, 0);
        assertTrue(ball.hasBall);
        assertEquals("starts at the mouth", 0, ball.stage());

        steps(10, 1, 0, 0);
        assertTrue("and moves along", ball.progress > 0);
    }

    @Test
    public void itWaitsOnTheRampUntilTheRampRuns() {
        steps(100, 1, 0, 0);

        assertTrue(ball.hasBall);
        assertEquals("handed to the ramp", 1, ball.stage());
        assertFalse(ball.takeShot());
    }

    @Test
    public void reversingSpitsItBackOut() {
        steps(10, 1, 0, 0);
        steps(60, -0.5, -1, 0);

        assertFalse("a ball driven backwards past the mouth is gone", ball.hasBall);
    }

    @Test
    public void stoppedWheelsDoNotTakeTheBall() {
        steps(200, 1, 1, 0);

        assertTrue("still in the robot", ball.hasBall);
        assertEquals("at the end of the ramp", 1, ball.stage());
        assertFalse(ball.takeShot());
    }

    @Test
    public void oneBallOutIsOneShot() {
        steps(30, 1, 0, 0);   // up to the ramp
        steps(60, 0, 1, 1);   // and out, with the intake off so nothing follows

        assertTrue("the ball left", ball.takeShot());
        assertFalse("and only once", ball.takeShot());
    }
}

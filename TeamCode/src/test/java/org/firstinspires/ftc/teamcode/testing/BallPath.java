package org.firstinspires.ftc.teamcode.testing;

/**
 * Where the ball is, from the intake mouth to the flywheels.
 *
 * <p>The strip in the field view used to say which mechanisms were running.
 * That answers "is the intake on" but not "is anything actually going to come
 * out", which is the question a driver is really asking when the fire button
 * seems to do nothing.
 *
 * <p><b>One ball, no physics.</b> Progress is driven by whichever mechanism
 * currently owns it, at a rate that is a guess: nobody has timed a ball through
 * this robot. It cannot tell you how long a cycle takes. What it can show is the
 * order of events - that the intake hands a ball to the ramp and stops there,
 * that the ramp will push it at wheels that are not turning, that reversing spits
 * it back out, and that firing with an empty path does nothing.
 *
 * <p>A ball is assumed to be available whenever the intake runs forward, which
 * on a real field it very much is not.
 */
public final class BallPath {

    /** Fraction of the path the intake owns. The ramp has the rest. */
    private static final double INTAKE_END = 0.5;

    /** Path fractions per second at full power. Guesses, both. */
    private static final double INTAKE_RATE = 1.2;
    private static final double RAMP_RATE = 1.6;

    /** Below this a mechanism is not moving anything. */
    private static final double IDLE = 0.02;

    /** Whether a ball is anywhere in the path. */
    public boolean hasBall;

    /** 0 at the intake mouth, 1 leaving the flywheels. */
    public double progress;

    /** Set for the one step in which a ball left. Cleared by {@link #takeShot}. */
    private boolean shotPending;

    public void reset() {
        hasBall = false;
        progress = 0;
        shotPending = false;
    }

    /**
     * @param intake the intake's pull on a ball, positive towards the flywheels
     * @param ramp the ramp's push on a ball, positive towards the flywheels
     * @param flywheels how fast the wheels are really turning, as a fraction of
     *     free speed. Nothing in the robot code holds the ramp back for them, so
     *     a ball can arrive at wheels that are not turning, and stays there.
     */
    public void step(double intake, double ramp, double flywheels, double dtSec) {
        if (dtSec <= 0) return;

        if (!hasBall) {
            // Something to pick up, whenever the intake is running forward.
            if (intake > IDLE) {
                hasBall = true;
                progress = 0;
            }
            return;
        }

        double drive = progress < INTAKE_END ? intake : ramp;
        progress += drive * rateAt(progress) * dtSec;

        if (progress >= 1.0) {
            if (flywheels > IDLE) {
                // Out through the flywheels. Whether it goes anywhere near the
                // goal is not modelled - there is no trajectory here.
                shotPending = true;
                hasBall = false;
                progress = 0;
            } else {
                // Wheels that are not turning do not take a ball.
                progress = Math.nextDown(1.0);
            }
        } else if (progress < 0) {
            // Spat back out of the intake.
            hasBall = false;
            progress = 0;
        }
    }

    private static double rateAt(double progress) {
        return progress < INTAKE_END ? INTAKE_RATE : RAMP_RATE;
    }

    /** Which mechanism has the ball right now, for the label on the strip. */
    public int stage() {
        if (!hasBall) return -1;
        return progress < INTAKE_END ? 0 : 1;
    }

    /** True once per ball that made it out, and only once. */
    public boolean takeShot() {
        if (!shotPending) return false;
        shotPending = false;
        return true;
    }
}

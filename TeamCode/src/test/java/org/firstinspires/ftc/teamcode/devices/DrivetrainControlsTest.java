package org.firstinspires.ftc.teamcode.devices;

import static org.junit.Assert.assertEquals;

import org.firstinspires.ftc.teamcode.data.Vec2;
import org.firstinspires.ftc.teamcode.testing.DriverStation;
import org.junit.Test;

/** The driver-stick to drive-command mapping, and the two autonomous helpers. */
public class DrivetrainControlsTest {

    private static final double EPS = 1e-6;

    // ----------------------------------------------------------- teleop input

    @Test
    public void pushingTheStickForwardCommandsPositiveY() {
        // A gamepad reports stick-up as NEGATIVE y, which is why readControls
        // negates it. Getting this backwards drives the robot the wrong way.
        DriverStation ds = new DriverStation();
        ds.held1.left_stick_y = -1.0f;

        DrivetrainControls c = DrivetrainControls.readControls(ds.tick());

        assertEquals(1.0, c.translationYPower, EPS);
    }

    @Test
    public void stickXAndRotationPassThroughUnchanged() {
        DriverStation ds = new DriverStation();
        ds.held1.left_stick_x = 0.5f;
        ds.held1.right_stick_x = -0.25f;

        DrivetrainControls c = DrivetrainControls.readControls(ds.tick());

        assertEquals(0.5, c.translationXPower, EPS);
        assertEquals(-0.25, c.rotationPower, EPS);
    }

    @Test
    public void restingSticksProduceNoMotion() {
        DrivetrainControls c = DrivetrainControls.readControls(new DriverStation().tick());

        assertEquals(0.0, c.translationXPower, EPS);
        assertEquals(0.0, c.translationYPower, EPS);
        assertEquals(0.0, c.rotationPower, EPS);
    }

    // ------------------------------------------------------------ auto aiming

    @Test
    public void aimingPowerIsProportionalOutsideTolerance() {
        assertEquals(0.075 * 20, DrivetrainControls.computeAimingPower(20, 2), EPS);
    }

    @Test
    public void aimingPowerKeepsTheSignOfTheError() {
        assertEquals(-0.075 * 20, DrivetrainControls.computeAimingPower(-20, 2), EPS);
    }

    @Test
    public void aimingStopsInsideTolerance() {
        assertEquals(0.0, DrivetrainControls.computeAimingPower(1.5, 2), EPS);
        assertEquals("exactly at tolerance counts as arrived",
                0.0, DrivetrainControls.computeAimingPower(2.0, 2), EPS);
    }

    // -------------------------------------------------------- auto translation

    @Test
    public void translationPowerIsProportionalPerAxis() {
        DrivetrainControls c = DrivetrainControls.computeTranslationPower(
                new Vec2(2.0, -4.0), new Vec2(0.5, 0.5));

        assertEquals(0.75 * 2.0, c.translationXPower, EPS);
        assertEquals(0.75 * -4.0, c.translationYPower, EPS);
        assertEquals("translation never rotates", 0.0, c.rotationPower, EPS);
    }

    @Test
    public void eachAxisStopsOnItsOwnTolerance() {
        // Arrived in x, still far in y: x must stop while y keeps driving.
        DrivetrainControls c = DrivetrainControls.computeTranslationPower(
                new Vec2(0.1, 3.0), new Vec2(0.5, 0.5));

        assertEquals(0.0, c.translationXPower, EPS);
        assertEquals(0.75 * 3.0, c.translationYPower, EPS);
    }

    // -------------------------------------------------------------- combining

    @Test
    public void combiningAddsComponentwise() {
        DrivetrainControls c = new DrivetrainControls(0.1, 0.2, 0.3);
        c.combineControls(new DrivetrainControls(0.4, -0.2, 0.1));

        assertEquals(0.5, c.translationXPower, EPS);
        assertEquals(0.0, c.translationYPower, EPS);
        assertEquals(0.4, c.rotationPower, EPS);
    }

    @Test
    public void combinePowerMatchesCombineControls() {
        DrivetrainControls a = new DrivetrainControls(0.1, 0.2, 0.3);
        DrivetrainControls b = new DrivetrainControls(0.1, 0.2, 0.3);

        a.combinePower(0.4, -0.2, 0.1);
        b.combineControls(new DrivetrainControls(0.4, -0.2, 0.1));

        assertEquals(a.translationXPower, b.translationXPower, EPS);
        assertEquals(a.translationYPower, b.translationYPower, EPS);
        assertEquals(a.rotationPower, b.rotationPower, EPS);
    }

    @Test
    public void combiningCanExceedOne() {
        // Deliberate: Drivetrain normalises later, so the sum is allowed to
        // overshoot here. If that ever stops being true, this test should fail.
        DrivetrainControls c = new DrivetrainControls(0.8, 0, 0);
        c.combinePower(0.8, 0, 0);

        assertEquals(1.6, c.translationXPower, EPS);
    }
}

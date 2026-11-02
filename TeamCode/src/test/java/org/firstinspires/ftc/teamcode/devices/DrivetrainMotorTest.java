package org.firstinspires.ftc.teamcode.devices;

import static org.junit.Assert.assertEquals;
import static org.mockito.Mockito.atLeastOnce;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.data.Vec2;
import org.firstinspires.ftc.teamcode.testing.DesktopHardwareMap;
import org.junit.Before;
import org.junit.Test;
import org.mockito.ArgumentCaptor;

import java.util.HashMap;
import java.util.Map;

/**
 * What actually reaches the four drive motors.
 *
 * <p>Sign convention, which these tests lock in: the right-hand motors are
 * reversed in the robot configuration, not in code. So a command of
 * {@code (+, -, +, -)} for (front-left, front-right, rear-left, rear-right)
 * is the robot driving straight forward.
 */
public class DrivetrainMotorTest {

    private static final double EPS = 1e-6;

    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx rearLeft;
    private DcMotorEx rearRight;
    private Drivetrain drivetrain;

    @Before
    public void setUp() {
        frontLeft = mock(DcMotorEx.class);
        frontRight = mock(DcMotorEx.class);
        rearLeft = mock(DcMotorEx.class);
        rearRight = mock(DcMotorEx.class);
        // Built the way an OpMode builds it, from the hardware map, under the
        // names the robot configuration uses.
        Map<String, Object> devices = new HashMap<>();
        devices.put("lf", frontLeft);
        devices.put("rf", frontRight);
        devices.put("lb", rearLeft);
        devices.put("rb", rearRight);
        drivetrain = new Drivetrain(DesktopHardwareMap.of(devices), mock(Telemetry.class), false);
    }

    /** The most recent power commanded to a motor. */
    private static double lastPower(DcMotorEx motor) {
        ArgumentCaptor<Double> captor = ArgumentCaptor.forClass(Double.class);
        verify(motor, atLeastOnce()).setPower(captor.capture());
        return captor.getValue();
    }

    private void assertPowers(double fl, double fr, double rl, double rr) {
        assertEquals("front left", fl, lastPower(frontLeft), EPS);
        assertEquals("front right", fr, lastPower(frontRight), EPS);
        assertEquals("rear left", rl, lastPower(rearLeft), EPS);
        assertEquals("rear right", rr, lastPower(rearRight), EPS);
    }

    // ------------------------------------------------------------------- init

    @Test
    public void initLeavesEveryMotorStoppedAndBraked() {
        assertPowers(0, 0, 0, 0);
        verify(frontLeft).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        verify(frontLeft).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        verify(rearRight).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        verify(rearRight).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // -------------------------------------------------------------- kinematics

    @Test
    public void forwardDrivesAllFourWheelsTheSameWay() {
        drivetrain.run(new DrivetrainControls(0, 1, 0));

        assertPowers(1, -1, 1, -1);
    }

    @Test
    public void strafeSplitsTheWheelsDiagonally() {
        drivetrain.run(new DrivetrainControls(1, 0, 0));

        assertPowers(1, 1, -1, -1);
    }

    @Test
    public void rotationDrivesBothSidesOpposite() {
        drivetrain.run(new DrivetrainControls(0, 0, 1));

        assertPowers(1, 1, 1, 1);
    }

    @Test
    public void rotateHelperMatchesARotationOnlyCommand() {
        drivetrain.rotate(-0.5);

        assertPowers(-0.5, -0.5, -0.5, -0.5);
    }

    // ----------------------------------------------------------- normalisation

    @Test
    public void combinedCommandsAreScaledToFitInsideFullPower() {
        // Raw: fl=3, fr=1, rl=1, rr=-1. Everything divides by 3 so the largest
        // wheel sits at exactly 1.0 and the ratios between wheels survive.
        drivetrain.run(new DrivetrainControls(1, 1, 1));

        assertPowers(1.0, 1.0 / 3, 1.0 / 3, -1.0 / 3);
    }

    @Test
    public void commandsThatAlreadyFitAreLeftAlone() {
        drivetrain.run(new DrivetrainControls(0.2, 0.3, 0.1));

        assertPowers(0.6, 0.0, 0.2, -0.4);
    }

    // ------------------------------------------------------------ translate()

    @Test
    public void translateNormalisesTheDirectionBeforeScaling() {
        // (3,4) has length 5, so this is a half-power push along that heading.
        drivetrain.translate(new Vec2(3, 4), 0.5);

        double x = 0.5 * 3 / 5;
        double y = 0.5 * 4 / 5;
        assertPowers(y + x, -y + x, y - x, -y - x);
    }

    // ------------------------------------------------------------- setPowers()

    @Test
    public void setPowersIgnoresAMalformedArray() {
        drivetrain.setPowers(new double[]{1, 1});
        drivetrain.setPowers(null);

        assertPowers(0, 0, 0, 0);
    }

    @Test
    public void setPowersMapsToMotorsInFrontLeftFrontRightRearRightRearLeftOrder() {
        // The internal array order is unusual; this is the contract callers rely on.
        drivetrain.setPowers(new double[]{0.1, 0.2, 0.3, 0.4});

        assertPowers(0.1, 0.2, 0.4, 0.3);
    }
}

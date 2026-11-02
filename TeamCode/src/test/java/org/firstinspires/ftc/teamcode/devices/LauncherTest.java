package org.firstinspires.ftc.teamcode.devices;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertThrows;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.eq;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.testing.DesktopHardwareMap;
import org.junit.Before;
import org.junit.Test;

import java.util.HashMap;
import java.util.Map;

/** The two flywheels, which this branch drives open loop with setPower. */
public class LauncherTest {

    private static final double EPS = 1e-6;

    private DcMotorEx left;
    private DcMotorEx right;
    private Telemetry telemetry;
    private Launcher launcher;

    @Before
    public void setUp() {
        left = mock(DcMotorEx.class);
        right = mock(DcMotorEx.class);
        telemetry = mock(Telemetry.class);
        Map<String, Object> devices = new HashMap<>();
        devices.put("leftLauncher", left);
        devices.put("rightLauncher", right);
        launcher = new Launcher(DesktopHardwareMap.of(devices), telemetry);
    }

    @Test
    public void theTwoWheelsFaceOppositeWays() {
        // They sit either side of the ball, so they must counter-rotate.
        verify(left).setDirection(DcMotor.Direction.FORWARD);
        verify(right).setDirection(DcMotor.Direction.REVERSE);
    }

    @Test
    public void initLeavesThemStoppedOpenLoopAndFloating() {
        verify(left).setPower(0.0);
        verify(right).setPower(0.0);
        verify(left).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verify(left).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Test
    public void spinDrivesBothWheelsAtOnePower() {
        launcher.spin(0.6);

        verify(left).setPower(0.6);
        verify(right).setPower(0.6);
    }

    @Test
    public void runTakesTheWheelPowerFromTheControls() {
        LauncherControls controls = new LauncherControls();
        controls.wheelPower = 0.4;

        launcher.run(controls);

        verify(left).setPower(0.4);
        verify(right).setPower(0.4);
        verify(telemetry).addData(eq("RPM-left"), (Object) any());
        verify(telemetry).addData(eq("RPM-right"), (Object) any());
    }

    @Test
    public void spinToVelocityIsTheClosedLoopPath() {
        launcher.spinToVelocity(1000);

        verify(left).setVelocity(1000);
        verify(right).setVelocity(1000);
    }

    @Test
    public void rpmIsConvertedFromTicksPerSecond() {
        // One revolution per second must read as 60 RPM.
        when(left.getVelocity()).thenReturn(Launcher.RRP);
        when(right.getVelocity()).thenReturn(Launcher.RRP * 2);

        assertEquals(60.0, launcher.getLeftWheelRPM(), EPS);
        assertEquals(120.0, launcher.getRightWheelRPM(), EPS);
    }

    @Test
    public void abortThrowsBecauseTheFeederIsNeverSetUp() {
        /*
         * The feeder field survived the move to the ramp but nothing assigns it
         * any more, so abort(), fire() and resetFeeder() all dereference null.
         * Nothing live on this branch calls them. Remove them, or the field.
         */
        assertThrows(NullPointerException.class, launcher::abort);
    }
}

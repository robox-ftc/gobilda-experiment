package org.firstinspires.ftc.teamcode.devices;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.testing.DesktopHardwareMap;
import org.junit.Before;
import org.junit.Test;

import java.util.Collections;

public class RampTest {

    private DcMotorEx wheel;
    private Ramp ramp;

    @Before
    public void setUp() {
        wheel = mock(DcMotorEx.class);
        ramp = new Ramp(DesktopHardwareMap.of(Collections.singletonMap("ramp", wheel)),
                mock(Telemetry.class));
    }

    @Test
    public void initSetsUpTheMotor() {
        verify(wheel).setDirection(DcMotorSimple.Direction.FORWARD);
        verify(wheel).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verify(wheel).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Test
    public void spinNegatesThePowerOnItsWayToTheMotor() {
        ramp.spin(0.6);

        verify(wheel).setPower(-0.6);
    }

    @Test
    public void runTakesTheRampPowerFromTheControls() {
        LauncherControls controls = new LauncherControls();
        controls.rampPower = -0.5;

        ramp.run(controls);

        verify(wheel).setPower(0.5);
    }
}

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

public class IntakeTest {

    private DcMotorEx wheel;
    private Intake intake;

    @Before
    public void setUp() {
        wheel = mock(DcMotorEx.class);
        intake = new Intake(DesktopHardwareMap.of(Collections.singletonMap("intake", wheel)),
                mock(Telemetry.class));
    }

    @Test
    public void initRunsTheMotorForward() {
        // FORWARD, with power passed straight through, so spin(+1) pulls a ball
        // in. main and october-new-robot-test get the same result the other
        // way round: REVERSE, with spin() negating. Mixing the two reverses it.
        verify(wheel).setDirection(DcMotorSimple.Direction.FORWARD);
        verify(wheel).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verify(wheel).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Test
    public void spinPassesPowerStraightThrough() {
        intake.spin(0.6);

        verify(wheel).setPower(0.6);
    }

    @Test
    public void runClampsToTheLegalPowerRange() {
        intake.run(2.5);
        verify(wheel).setPower(1.0);

        intake.run(-2.5);
        verify(wheel).setPower(-1.0);
    }
}

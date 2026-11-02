package org.firstinspires.ftc.teamcode.devices;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.testing.DesktopHardwareMap;
import org.junit.Before;
import org.junit.Test;

import java.util.Collections;

/**
 * The hood servo. It asks the hardware map for a CRServoImplEx, a class rather
 * than an interface, so the stand-in is a Mockito mock.
 *
 * <p>Its servo field and methods are static: a second Hood replaces the servo
 * every Hood uses. Harmless with one robot; worth knowing before writing a test
 * that builds two.
 */
public class HoodTest {

    private CRServoImplEx servo;

    @Before
    public void setUp() {
        servo = mock(CRServoImplEx.class);
        new Hood(DesktopHardwareMap.of(Collections.singletonMap("hood", servo)), mock(Telemetry.class));
    }

    @Test
    public void initSetsTheDirection() {
        verify(servo).setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Test
    public void runClampsToTheLegalPowerRange() {
        Hood.run(2.5);
        verify(servo).setPower(1.0);

        Hood.run(-0.3);
        verify(servo).setPower(-0.3);
    }
}

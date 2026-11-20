package org.firstinspires.ftc.teamcode.devices;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.junit.Test;

public class BatteryCompensationTest {

    private static BatteryCompensation at(double volts) {
        VoltageSensor sensor = mock(VoltageSensor.class);
        when(sensor.getVoltage()).thenReturn(volts);
        return new BatteryCompensation(sensor);
    }

    @Test
    public void aLowBatteryGetsMorePowerAndAFullOneLess() {
        assertEquals(12.0 / 11.0, at(11.0).factor(), 1e-12);
        assertEquals(12.0 / 13.2, at(13.2).factor(), 1e-12);
        assertEquals(1.0, at(12.0).factor(), 1e-12);
    }

    @Test
    public void withoutAUsableReadingNothingIsCorrected() {
        assertEquals(1.0, new BatteryCompensation((VoltageSensor) null).factor(), 0);
        assertEquals("a disconnected sensor reads 0", 1.0, at(0).factor(), 0);
        assertTrue(Double.isNaN(at(0).volts()));
    }
}

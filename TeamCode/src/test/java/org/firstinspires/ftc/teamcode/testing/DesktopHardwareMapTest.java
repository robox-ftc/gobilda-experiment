package org.firstinspires.ftc.teamcode.testing;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.fail;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.junit.Test;

import java.util.LinkedHashMap;
import java.util.Map;

/** The stand-in HardwareMap answers lookups the way the SDK's does. */
public class DesktopHardwareMapTest {

    private final FakeHardware.Motor lf = FakeHardware.motor("lf");
    private final FakeHardware.Motor intake = FakeHardware.motor("intake");
    private final HardwareMap map;

    public DesktopHardwareMapTest() {
        Map<String, Object> devices = new LinkedHashMap<>();
        devices.put("lf", lf.device);
        devices.put("intake", intake.device);
        map = DesktopHardwareMap.of(devices);
    }

    @Test
    public void getReturnsTheDeviceRegisteredUnderThatName() {
        assertSame(lf.device, map.get(DcMotorEx.class, "lf"));
        assertSame("a supertype finds it too", lf.device, map.get(DcMotor.class, "lf"));
    }

    @Test
    public void namesAreTrimmedAsTheSdkTrimsThem() {
        assertSame(lf.device, map.get(DcMotorEx.class, " lf "));
    }

    @Test
    public void anUnknownNameThrowsTheSdksException() {
        try {
            map.get(DcMotorEx.class, "turret");
            fail();
        } catch (IllegalArgumentException e) {
            assertEquals("Unable to find a hardware device with name \"turret\" and type DcMotorEx",
                    e.getMessage());
        }
    }

    @Test(expected = IllegalArgumentException.class)
    public void theWrongTypeIsNotFound() {
        map.get(CRServo.class, "lf");
    }

    @Test
    public void tryGetReturnsNullInstead() {
        assertNull(map.tryGet(DcMotorEx.class, "turret"));
        assertSame(intake.device, map.tryGet(DcMotorEx.class, "intake"));
    }

    @Test
    public void getAllFindsEveryDeviceOfAType() {
        assertEquals(2, map.getAll(DcMotor.class).size());
        assertEquals(0, map.getAll(CRServo.class).size());
    }
}

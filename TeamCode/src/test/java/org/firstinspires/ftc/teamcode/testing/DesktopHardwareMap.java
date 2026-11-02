package org.firstinspires.ftc.teamcode.testing;

import static org.mockito.Mockito.withSettings;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.mockito.Answers;
import org.mockito.Mockito;
import org.mockito.invocation.InvocationOnMock;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * A HardwareMap for the desktop. Robot code calls {@code hardwareMap.get(type,
 * name)} exactly as it does on the robot and gets back whatever stand-in was
 * registered under that name, so device classes and OpModes run unmodified.
 *
 * <p>The SDK's own HardwareMap can be constructed off-robot, but its get() runs
 * {@code Device}'s static initialiser, which reaches for the Android app through
 * {@code AppUtil} and fails. So this is a Mockito stand-in, created without
 * running the SDK constructor, that answers the lookups the way the real one
 * does (SDK 11.0.0 bytecode): names are trimmed, an unknown name or a device of
 * the wrong type throws IllegalArgumentException with the SDK's own message,
 * and tryGet() returns null instead. Robot code that catches that exception to
 * carry on without a device therefore takes the same path here.
 */
public final class DesktopHardwareMap {

    private DesktopHardwareMap() {}

    /** @param devices configuration name to device, as in the robot configuration */
    public static HardwareMap of(Map<String, ?> devices) {
        Map<String, Object> byName = new LinkedHashMap<>(devices);
        return Mockito.mock(HardwareMap.class,
                withSettings().defaultAnswer(invocation -> answer(byName, invocation)));
    }

    private static Object answer(Map<String, Object> byName, InvocationOnMock invocation) throws Throwable {
        String method = invocation.getMethod().getName();
        Object[] args = invocation.getArguments();

        boolean byTypeAndName = args.length == 2 && args[0] instanceof Class && args[1] instanceof String;
        if (byTypeAndName && (method.equals("get") || method.equals("tryGet"))) {
            Class<?> type = (Class<?>) args[0];
            String name = ((String) args[1]).trim();
            Object device = byName.get(name);
            if (type.isInstance(device)) return device;
            if (method.equals("tryGet")) return null;
            throw new IllegalArgumentException(String.format(
                    "Unable to find a hardware device with name \"%s\" and type %s",
                    name, type.getSimpleName()));
        }
        if (method.equals("getAll") && args.length == 1 && args[0] instanceof Class) {
            List<Object> all = new ArrayList<>();
            for (Object device : byName.values()) {
                if (((Class<?>) args[0]).isInstance(device)) all.add(device);
            }
            return all;
        }
        return Answers.RETURNS_DEFAULTS.answer(invocation);
    }
}

package org.firstinspires.ftc.teamcode.testing;

import static org.mockito.Mockito.withSettings;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.mockito.Answers;
import org.mockito.MockedConstruction;
import org.mockito.Mockito;
import org.mockito.invocation.InvocationOnMock;

import java.util.Collections;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;

/**
 * Robot time on the desktop.
 *
 * <p>Robot code measures time with {@link ElapsedTime}, which reads
 * {@code System.nanoTime()}. Left alone, that makes a 30-second autonomous take
 * 30 seconds to test, and makes the outcome depend on how busy the PC is. While
 * {@link #install()} is in effect, every ElapsedTime constructed on this thread -
 * including the ones an OpMode creates in its field initialisers - reads this
 * clock instead, and the clock moves only when {@link #advance} says so.
 *
 * <p>This is Mockito's {@code mockConstruction}, so it has the same two limits:
 * it covers construction on the installing thread only, and it does not reach
 * {@code System.nanoTime()} called directly. {@code OpMode.getRuntime()} does
 * that; nothing in TeamCode calls it today.
 */
public final class SimClock {

    private long nanos;

    public long nanos() {
        return nanos;
    }

    public double seconds() {
        return nanos / 1e9;
    }

    public void advance(double seconds) {
        if (seconds < 0) throw new IllegalArgumentException("time only goes forward: " + seconds);
        nanos += Math.round(seconds * 1e9);
    }

    /** Points ElapsedTime at this clock until the returned scope is closed. */
    public MockedConstruction<ElapsedTime> install() {
        Map<Object, Timer> timers = Collections.synchronizedMap(new IdentityHashMap<>());
        return Mockito.mockConstruction(ElapsedTime.class,
                context -> withSettings().defaultAnswer(invocation -> answer(timers, invocation)),
                (mock, context) -> timers.put(mock, new Timer(context.arguments())));
    }

    /** What one ElapsedTime remembers: where it started, and how it reports time(). */
    private final class Timer {
        long start;
        final boolean millis;

        Timer(List<?> constructorArguments) {
            long s = nanos;
            boolean ms = false;
            if (constructorArguments.size() == 1) {
                Object arg = constructorArguments.get(0);
                // ElapsedTime(long) takes a start time in nanoseconds.
                if (arg instanceof Long) s = (Long) arg;
                if (arg == ElapsedTime.Resolution.MILLISECONDS) ms = true;
            }
            start = s;
            millis = ms;
        }
    }

    private Object answer(Map<Object, Timer> timers, InvocationOnMock invocation) throws Throwable {
        Timer t = timers.computeIfAbsent(invocation.getMock(), k -> new Timer(Collections.emptyList()));
        long elapsed = nanos - t.start;
        Object[] args = invocation.getArguments();
        switch (invocation.getMethod().getName()) {
            case "reset":
                t.start = nanos;
                return null;
            case "nsNow":
                return nanos;
            case "now":
                return ((TimeUnit) args[0]).convert(nanos, TimeUnit.NANOSECONDS);
            case "nanoseconds":
                return elapsed;
            case "milliseconds":
                return elapsed / 1e6;
            case "seconds":
                return elapsed / 1e9;
            case "time":
                if (args.length == 1) return ((TimeUnit) args[0]).convert(elapsed, TimeUnit.NANOSECONDS);
                return t.millis ? elapsed / 1e6 : elapsed / 1e9;
            case "startTime":
                return t.millis ? t.start / 1e6 : t.start / 1e9;
            case "startTimeNanoseconds":
                return t.start;
            case "getResolution":
                return t.millis ? ElapsedTime.Resolution.MILLISECONDS : ElapsedTime.Resolution.SECONDS;
            case "toString":
                return String.format("%.4f seconds", elapsed / 1e9);
            default:
                return Answers.RETURNS_DEFAULTS.answer(invocation);
        }
    }
}

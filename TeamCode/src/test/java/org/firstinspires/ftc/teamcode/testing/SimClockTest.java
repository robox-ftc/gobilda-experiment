package org.firstinspires.ftc.teamcode.testing;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.junit.Test;
import org.mockito.MockedConstruction;
import org.mockito.Mockito;

import java.util.concurrent.TimeUnit;

public class SimClockTest {

    private final SimClock clock = new SimClock();

    @Test
    public void anElapsedTimeReadsTheClockWhileItIsInstalled() {
        try (MockedConstruction<ElapsedTime> ignored = clock.install()) {
            ElapsedTime timer = new ElapsedTime();
            clock.advance(1.5);

            assertEquals(1.5, timer.seconds(), 1e-12);
            assertEquals(1500.0, timer.milliseconds(), 1e-9);
            assertEquals(1_500_000_000L, timer.nanoseconds());
            assertEquals(1500L, timer.time(TimeUnit.MILLISECONDS));
        }
    }

    @Test
    public void resetStartsOver() {
        try (MockedConstruction<ElapsedTime> ignored = clock.install()) {
            ElapsedTime timer = new ElapsedTime();
            clock.advance(2);
            timer.reset();
            clock.advance(0.25);

            assertEquals(0.25, timer.seconds(), 1e-12);
        }
    }

    @Test
    public void eachTimerKeepsItsOwnStart() {
        try (MockedConstruction<ElapsedTime> ignored = clock.install()) {
            ElapsedTime early = new ElapsedTime();
            clock.advance(1);
            ElapsedTime late = new ElapsedTime();
            clock.advance(1);

            assertEquals(2.0, early.seconds(), 1e-12);
            assertEquals(1.0, late.seconds(), 1e-12);
        }
    }

    @Test
    public void millisecondResolutionReportsMilliseconds() {
        try (MockedConstruction<ElapsedTime> ignored = clock.install()) {
            ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            clock.advance(0.25);

            assertEquals(250.0, timer.time(), 1e-9);
            assertEquals(ElapsedTime.Resolution.MILLISECONDS, timer.getResolution());
        }
    }

    @Test
    public void outsideTheScopeElapsedTimeIsTheRealOne() {
        try (MockedConstruction<ElapsedTime> ignored = clock.install()) {
            assertTrue(Mockito.mockingDetails(new ElapsedTime()).isMock());
        }
        assertFalse(Mockito.mockingDetails(new ElapsedTime()).isMock());
    }

    @Test(expected = IllegalArgumentException.class)
    public void timeOnlyGoesForward() {
        clock.advance(-0.1);
    }
}

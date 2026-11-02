package org.firstinspires.ftc.teamcode.testing;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.Test;

import java.util.List;

/**
 * The harness shows telemetry the way a Driver Station would. TelemetryImpl
 * (SDK 11.0.0) transmits on update() only once msTransmissionInterval, 250 ms by
 * default, has passed since the last transmission, and with auto-clear on it
 * clears the lines either way - so a line added between transmissions is lost.
 */
public class TelemetryRateTest {

    private final SimClock clock = new SimClock();
    private final FakeHardware.TelemetryRecorder recorder = new FakeHardware.TelemetryRecorder(clock::nanos);
    private final Telemetry telemetry = recorder.device;

    private void loop(String line, double seconds) {
        telemetry.addLine(line);
        telemetry.update();
        clock.advance(seconds);
    }

    @Test
    public void theFirstFrameGoesOutImmediately() {
        loop("one", 0.02);

        assertEquals(List.of("one"), recorder.visible());
    }

    @Test
    public void whatIsOnScreenDoesNotChangeBetweenTransmissions() {
        loop("one", 0.02);
        List<String> first = recorder.visible();
        for (int i = 0; i < 10; i++) loop("later " + i, 0.02); // 200 ms

        assertSame(first, recorder.visible());
    }

    @Test
    public void theNextFrameGoesOutOnceTheIntervalHasPassed() {
        loop("one", 0.02);
        for (int i = 0; i < 13; i++) loop("later " + i, 0.02); // 260 ms

        assertEquals(1, recorder.visible().size());
        assertTrue(recorder.visible().get(0).startsWith("later"));
    }

    @Test
    public void aLineAddedBetweenTransmissionsIsNeverShown() {
        loop("one", 0.1);
        loop("missed", 0.02);
        for (int i = 0; i < 20; i++) loop("", 0.02);

        assertFalse(recorder.visible().contains("missed"));
    }

    @Test
    public void aFrameIsOneLoopsLinesNotAnAccumulation() {
        recorder.setMsTransmissionInterval(0);
        for (int i = 0; i < 20; i++) loop("line", 0.02);

        assertEquals(1, recorder.visible().size());
    }
}

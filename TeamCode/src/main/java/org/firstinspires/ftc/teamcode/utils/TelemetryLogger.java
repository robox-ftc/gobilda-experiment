package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/**
 * Sends telemetry to the Driver Station like normal, and also writes every
 * {@link #update()} to a text file with a timestamp, so a run can be read back
 * after the match.
 *
 * <p>Use it in place of {@code telemetry} for the lines you want recorded:
 * <pre>
 *     log = new TelemetryLogger(telemetry, "BIOBUZZAutoOdo");   // in init()
 *     log.startClock();                                       // in start()
 *     log.addData("Current X", x);                            // in loop()
 *     log.update();
 *     log.close();                                            // in stop()
 * </pre>
 *
 * <p>Each update becomes one line: seconds since {@link #startClock()} (or
 * since the logger was made, before that), then every caption and value:
 * <pre>
 *     [   1.234] Current X: 12.000 | Current Y: 3.500 | Remaining Tasks: 4
 * </pre>
 *
 * <p>On the robot the files go to {@code /sdcard/FIRST/logs} (pull them with
 * {@code adb pull /sdcard/FIRST/logs}). Off the robot, such as in the desktop
 * harness, they go to {@code build/telemetry-logs} under the working directory.
 * If the file can't be opened, telemetry still reaches the Driver Station and
 * nothing is recorded; the auto never stops because of the log.
 */
public class TelemetryLogger {
    private static final File ROBOT_LOG_DIR = new File("/sdcard/FIRST/logs");
    private static final File DESKTOP_LOG_DIR = new File("build/telemetry-logs");

    private final Telemetry telemetry;
    private final ElapsedTime clock = new ElapsedTime();
    private final StringBuilder entry = new StringBuilder();
    private BufferedWriter writer;
    private File file;

    public TelemetryLogger(Telemetry telemetry, String name) {
        this.telemetry = telemetry;
        Date now = new Date();
        String stamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(now);
        File dir = ROBOT_LOG_DIR.getParentFile().isDirectory() ? ROBOT_LOG_DIR : DESKTOP_LOG_DIR;
        try {
            if (!dir.isDirectory() && !dir.mkdirs()) {
                throw new IOException("could not create " + dir);
            }
            file = new File(dir, name + "_" + stamp + ".log");
            writer = new BufferedWriter(new FileWriter(file));
            writer.write("# " + name + " telemetry log, started "
                    + new SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.US).format(now));
            writer.newLine();
            writer.write("# [seconds] caption: value | caption: value ...");
            writer.newLine();
        } catch (IOException e) {
            writer = null;
            file = null;
            telemetry.addLine("Telemetry log not recording: " + e.getMessage());
        }
    }

    /** Restarts the timestamps at zero. Call this in start() so times match the match clock. */
    public void startClock() {
        clock.reset();
        writeLine("---------- START ----------");
    }

    /** Shows a caption and value on the Driver Station and records it. */
    public TelemetryLogger addData(String caption, Object value) {
        telemetry.addData(caption, value);
        separate();
        entry.append(caption).append(": ").append(format(value));
        return this;
    }

    /** Shows a line on the Driver Station and records it. */
    public TelemetryLogger addLine(String line) {
        telemetry.addLine(line);
        separate();
        entry.append(line);
        return this;
    }

    /** Sends everything added since the last update to the Driver Station, and writes it as one timestamped line. */
    public void update() {
        telemetry.update();
        if (entry.length() > 0) {
            writeLine(entry.toString());
            entry.setLength(0);
        }
    }

    /** Writes the last of the log to the file. Call this in stop(). */
    public void close() {
        if (writer == null) return;
        try {
            writer.close();
        } catch (IOException ignored) {
            // Nothing more can be done with a log that won't close.
        }
        writer = null;
    }

    /** The file being written, or null if the log couldn't be opened. */
    public File getFile() {
        return file;
    }

    private void writeLine(String text) {
        if (writer == null) return;
        try {
            writer.write(String.format(Locale.US, "[%8.3f] %s", clock.seconds(), text));
            writer.newLine();
            // Flushed every line so the log survives the robot losing power mid-auto.
            writer.flush();
        } catch (IOException e) {
            close();
        }
    }

    private void separate() {
        if (entry.length() > 0) entry.append(" | ");
    }

    private static String format(Object value) {
        if (value instanceof Double || value instanceof Float) {
            return String.format(Locale.US, "%.3f", ((Number) value).doubleValue());
        }
        return String.valueOf(value);
    }
}

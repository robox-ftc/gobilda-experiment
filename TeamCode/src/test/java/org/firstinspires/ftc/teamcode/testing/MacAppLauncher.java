package org.firstinspires.ftc.teamcode.testing;

import java.io.File;
import java.io.IOException;
import java.io.RandomAccessFile;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * Gets the test UI's window the keyboard on macOS.
 *
 * <p>Started by Gradle - from Android Studio or a terminal - the harness is a
 * background process, and macOS no longer lets a background process bring
 * itself to the front: {@code toFront()}, {@code requestFocus()} and
 * {@code Desktop.requestForeground} are all ignored (checked on macOS 27 with
 * Android Studio in front). What macOS does still bring forward is an app
 * started through LaunchServices, which is what {@code open} uses.
 *
 * <p>So on a Mac the harness started by Gradle does not open a window itself.
 * It wraps its own java command line in a small app bundle under
 * {@code build/desktop-harness}, starts that with {@code open}, passes the
 * app's output through to its own (so it still shows in the Run window), and
 * waits for it to close. When Gradle stops it - the Run window's stop button -
 * it closes the app too.
 */
final class MacAppLauncher {

    /** Set on the relaunched copy, so it runs the harness instead of relaunching again. */
    static final String RELAUNCHED = "harness.relaunched";

    private MacAppLauncher() {}

    /** True on a Mac, in the copy Gradle started, when the command line can be read back. */
    static boolean shouldRelaunch() {
        return System.getProperty("os.name", "").startsWith("Mac")
                && System.getProperty(RELAUNCHED) == null
                && !"false".equals(System.getProperty("harness.macApp"))
                && commandLine().isPresent();
    }

    /** This JVM's own command line: java, then every argument, as it was started. */
    private static Optional<List<String>> commandLine() {
        ProcessHandle.Info info = ProcessHandle.current().info();
        if (!info.command().isPresent() || !info.arguments().isPresent()) return Optional.empty();
        List<String> line = new ArrayList<>();
        line.add(info.command().get());
        for (String arg : info.arguments().get()) line.add(arg);
        return Optional.of(line);
    }

    /**
     * Runs the harness as an app started by {@code open}, and returns when it
     * has closed. Returns false, having done nothing, if it could not be set up;
     * the caller then carries on in this process, window in the background.
     */
    static boolean relaunch() {
        List<String> line = commandLine().orElse(null);
        if (line == null) return false;
        File dir = new File("build/desktop-harness").getAbsoluteFile();
        File app = new File(dir, "BIOBUZZ Test UI.app");
        File pidFile = new File(dir, "app.pid");
        File output = new File(dir, "app-output.log");
        try {
            writeBundle(app, line, pidFile);
            Files.deleteIfExists(pidFile.toPath());
            Files.write(output.toPath(), new byte[0]);
        } catch (IOException e) {
            System.err.println("Could not set up the test UI app (" + e.getMessage() + "); opening it here instead.");
            return false;
        }

        Process open;
        try {
            open = new ProcessBuilder("open", "-n", "-W",
                    "--stdout", output.getPath(), "--stderr", output.getPath(), app.getPath())
                    .inheritIO().start();
        } catch (IOException e) {
            System.err.println("Could not run open (" + e.getMessage() + "); opening it here instead.");
            return false;
        }

        // The Run window's stop button ends this process; take the app with it.
        Runtime.getRuntime().addShutdownHook(new Thread(() -> {
            stopApp(pidFile);
            open.destroy();
        }));

        Thread echo = new Thread(() -> follow(output, open), "test UI output");
        echo.setDaemon(true);
        echo.start();
        try {
            open.waitFor();
            echo.join(1000);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        // The app has closed; forget its pid so the shutdown hook cannot hit a reused one.
        try {
            Files.deleteIfExists(pidFile.toPath());
        } catch (IOException ignored) {
            // Harmless: the hook only reads it.
        }
        return true;
    }

    private static void writeBundle(File app, List<String> line, File pidFile) throws IOException {
        File macOs = new File(app, "Contents/MacOS");
        if (!macOs.isDirectory() && !macOs.mkdirs()) throw new IOException("cannot create " + macOs);
        Files.write(new File(app, "Contents/Info.plist").toPath(), (
                "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
                + "<!DOCTYPE plist PUBLIC \"-//Apple//DTD PLIST 1.0//EN\""
                + " \"http://www.apple.com/DTDs/PropertyList-1.0.dtd\">\n"
                + "<plist version=\"1.0\"><dict>\n"
                + "<key>CFBundleExecutable</key><string>launch</string>\n"
                + "<key>CFBundleIdentifier</key><string>org.firstinspires.ftc.teamcode.testui</string>\n"
                + "<key>CFBundleName</key><string>BIOBUZZ Test UI</string>\n"
                + "<key>CFBundlePackageType</key><string>APPL</string>\n"
                + "<key>NSHighResolutionCapable</key><true/>\n"
                + "</dict></plist>\n").getBytes(StandardCharsets.UTF_8));

        StringBuilder script = new StringBuilder("#!/bin/sh\n");
        script.append("cd ").append(quote(new File("").getAbsolutePath())).append(" || exit 1\n");
        script.append("echo $$ > ").append(quote(pidFile.getPath())).append('\n');
        script.append("exec ").append(quote(line.get(0)));
        script.append(' ').append(quote("-D" + RELAUNCHED + "=true"));
        for (String arg : line.subList(1, line.size())) script.append(' ').append(quote(arg));
        script.append('\n');
        File launch = new File(macOs, "launch");
        Files.write(launch.toPath(), script.toString().getBytes(StandardCharsets.UTF_8));
        if (!launch.setExecutable(true)) throw new IOException("cannot make " + launch + " executable");
    }

    /** Copies what the app writes to its log onto this process's output, until {@code open} ends. */
    private static void follow(File output, Process open) {
        long at = 0;
        byte[] buf = new byte[8192];
        try (RandomAccessFile in = new RandomAccessFile(output, "r")) {
            while (true) {
                boolean finished = !open.isAlive();
                long len = in.length();
                if (len > at) {
                    in.seek(at);
                    int n;
                    while (at < len && (n = in.read(buf, 0, (int) Math.min(buf.length, len - at))) > 0) {
                        System.out.write(buf, 0, n);
                        at += n;
                    }
                    System.out.flush();
                }
                if (finished) return;
                Thread.sleep(150);
            }
        } catch (IOException | InterruptedException ignored) {
            // Output is a convenience; the app runs regardless.
        }
    }

    private static void stopApp(File pidFile) {
        try {
            long pid = Long.parseLong(new String(Files.readAllBytes(pidFile.toPath()), StandardCharsets.UTF_8).trim());
            ProcessHandle.of(pid).ifPresent(ProcessHandle::destroy);
        } catch (IOException | NumberFormatException ignored) {
            // Never started, or already gone.
        }
    }

    /** Single-quotes for /bin/sh. */
    private static String quote(String s) {
        return "'" + s.replace("'", "'\\''") + "'";
    }
}

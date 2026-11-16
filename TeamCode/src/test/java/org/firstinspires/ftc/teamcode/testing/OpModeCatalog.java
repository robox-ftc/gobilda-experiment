package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.File;
import java.io.IOException;
import java.net.JarURLConnection;
import java.net.URISyntaxException;
import java.net.URL;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Enumeration;
import java.util.List;
import java.util.Locale;
import java.util.TreeSet;
import java.util.jar.JarEntry;
import java.util.jar.JarFile;

/**
 * The OpModes the Driver Station would list, found on the classpath.
 *
 * <p>Like the Driver Station, only classes annotated {@link TeleOp} or
 * {@link Autonomous} and not {@link Disabled} are listed, so a new OpMode file
 * shows up in the harness without touching it. A {@link LinearOpMode} is listed
 * but marked unsupported: it runs on its own thread, which {@link OpModeRunner}
 * does not do.
 */
public final class OpModeCatalog {

    /** The package TeamCode OpModes live in, subpackages included. */
    static final String PACKAGE = "org.firstinspires.ftc.teamcode";

    public enum Kind {
        TELEOP("TeleOp"), AUTONOMOUS("Auto");

        final String label;

        Kind(String label) {
            this.label = label;
        }
    }

    /** One OpMode as the picker shows it. */
    public static final class Entry {
        public final Class<? extends OpMode> type;
        public final String name;
        public final Kind kind;
        /** Null when the harness can run it; otherwise why not. */
        public final String unsupported;

        Entry(Class<? extends OpMode> type, String name, Kind kind, String unsupported) {
            this.type = type;
            this.name = name;
            this.kind = kind;
            this.unsupported = unsupported;
        }

        public boolean runnable() {
            return unsupported == null;
        }

        @Override public String toString() {
            return name + " (" + type.getSimpleName() + ", " + kind.label + ")";
        }
    }

    private OpModeCatalog() {}

    /** TeleOps first, then autonomous, each by Driver Station name. */
    public static List<Entry> discover() {
        ClassLoader loader = Thread.currentThread().getContextClassLoader();
        if (loader == null) loader = OpModeCatalog.class.getClassLoader();
        List<Entry> found = new ArrayList<Entry>();
        for (String className : classNames(loader)) {
            Entry entry = entryFor(className, loader);
            if (entry != null) found.add(entry);
        }
        Collections.sort(found, (a, b) -> {
            if (a.kind != b.kind) return a.kind.compareTo(b.kind);
            return a.name.toLowerCase(Locale.ROOT).compareTo(b.name.toLowerCase(Locale.ROOT));
        });
        return found;
    }

    /** The entry for {@code type}, or a plain one if the Driver Station would not list it. */
    public static Entry describe(Class<? extends OpMode> type) {
        Entry entry = entryFor(type.getName(), type.getClassLoader());
        if (entry != null) return entry;
        Kind kind = type.isAnnotationPresent(TeleOp.class) ? Kind.TELEOP : Kind.AUTONOMOUS;
        return new Entry(type, OpModeRunner.displayName(type), kind, unsupportedReason(type));
    }

    private static Entry entryFor(String className, ClassLoader loader) {
        Class<?> found;
        try {
            // Not initialized: reading annotations must not run an OpMode's statics.
            found = Class.forName(className, false, loader);
        } catch (Throwable notLoadable) {
            return null; // e.g. it needs an Android class the desktop does not have
        }
        if (!OpMode.class.isAssignableFrom(found)) return null;
        if (found.isAnnotationPresent(Disabled.class)) return null;
        Kind kind;
        if (found.isAnnotationPresent(TeleOp.class)) kind = Kind.TELEOP;
        else if (found.isAnnotationPresent(Autonomous.class)) kind = Kind.AUTONOMOUS;
        else return null;
        Class<? extends OpMode> type = found.asSubclass(OpMode.class);
        return new Entry(type, OpModeRunner.displayName(type), kind, unsupportedReason(type));
    }

    private static String unsupportedReason(Class<? extends OpMode> type) {
        if (LinearOpMode.class.isAssignableFrom(type)) return "LinearOpMode is not supported";
        if (java.lang.reflect.Modifier.isAbstract(type.getModifiers())) return "abstract";
        return null;
    }

    /**
     * Top-level class names under {@link #PACKAGE}, from directories and jars.
     *
     * <p>Both the classpath and the loader are asked: the Android build puts
     * TeamCode in a jar with no directory entries, which
     * {@code getResources} on a package path never finds.
     */
    private static TreeSet<String> classNames(ClassLoader loader) {
        TreeSet<String> names = new TreeSet<String>();
        String path = PACKAGE.replace('.', '/');
        for (String element : System.getProperty("java.class.path", "").split(File.pathSeparator)) {
            if (element.isEmpty()) continue;
            File file = new File(element);
            if (file.isDirectory()) {
                scanDirectory(new File(file, path), PACKAGE, names);
            } else if (file.isFile() && element.endsWith(".jar")) {
                try (JarFile jar = new JarFile(file)) {
                    scanJar(jar, path, names);
                } catch (IOException e) {
                    // An unreadable classpath entry just contributes nothing.
                }
            }
        }
        Enumeration<URL> roots;
        try {
            roots = loader.getResources(path);
        } catch (IOException e) {
            return names;
        }
        while (roots.hasMoreElements()) {
            URL root = roots.nextElement();
            try {
                if ("file".equals(root.getProtocol())) {
                    scanDirectory(new File(root.toURI()), PACKAGE, names);
                } else if ("jar".equals(root.getProtocol())) {
                    scanJar(((JarURLConnection) root.openConnection()).getJarFile(), path, names);
                }
            } catch (IOException | URISyntaxException | ClassCastException e) {
                // An unreadable classpath entry just contributes nothing.
            }
        }
        return names;
    }

    private static void scanDirectory(File dir, String pkg, TreeSet<String> names) {
        File[] files = dir.listFiles();
        if (files == null) return;
        for (File file : files) {
            String fileName = file.getName();
            if (file.isDirectory()) {
                scanDirectory(file, pkg + "." + fileName, names);
            } else if (isTopLevelClass(fileName)) {
                names.add(pkg + "." + fileName.substring(0, fileName.length() - ".class".length()));
            }
        }
    }

    private static void scanJar(JarFile jar, String path, TreeSet<String> names) {
        Enumeration<JarEntry> entries = jar.entries();
        while (entries.hasMoreElements()) {
            String entryName = entries.nextElement().getName();
            if (!entryName.startsWith(path + "/")) continue;
            String fileName = entryName.substring(entryName.lastIndexOf('/') + 1);
            if (isTopLevelClass(fileName)) {
                names.add(entryName.substring(0, entryName.length() - ".class".length()).replace('/', '.'));
            }
        }
    }

    private static boolean isTopLevelClass(String fileName) {
        return fileName.endsWith(".class") && fileName.indexOf('$') < 0;
    }
}

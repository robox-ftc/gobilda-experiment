package org.firstinspires.ftc.teamcode.testing;

import org.firstinspires.ftc.teamcode.devices.GamePadReadings;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.GraphicsEnvironment;
import java.awt.RenderingHints;
import java.awt.Shape;
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.awt.geom.AffineTransform;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;
import java.awt.geom.Path2D;
import java.awt.geom.Point2D;
import java.awt.geom.RoundRectangle2D;
import java.util.ArrayDeque;
import java.util.Arrays;
import java.util.ArrayList;
import java.util.Deque;
import java.util.List;
import java.util.Locale;

import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.SwingUtilities;

/**
 * A window showing what a running OpMode just told the robot to do.
 *
 * <p>Everything drawn here comes off {@link DesktopRobot}'s stand-in hardware,
 * so it is the real Drivetrain, Intake, Launcher and Ramp being watched, and
 * the real control mapping deciding what they are told. Position
 * comes from {@link RobotSim}, which is dead reckoning - read the caveat on
 * that class before trusting a distance.
 *
 * <p>Run it with {@code gradlew :TeamCode:gamepadGui}. Closing the window, or
 * BACK + START on the pad, ends the harness.
 */
public final class RobotView {

    private static final int TRAIL_POINTS = 400;
    private static final int TELEMETRY_LINES = 12;

    /** How long a fired-shot marker stays on the field. */
    private static final double SHOT_FADE_SECONDS = 1.2;

    /**
     * A moment a ball left the flywheels. There is no trajectory model here,
     * so it is drawn as a short ray along the robot's heading and faded out:
     * the launcher is fixed, so that is where a shot goes.
     */
    private static final class Shot {
        final double x, y, bearing;
        double age;
        Shot(double x, double y, double bearing) {
            this.x = x; this.y = y; this.bearing = bearing;
        }
    }

    /** Below this a command reads as "off" rather than as a direction. */
    private static final double IDLE = 0.02;

    /** How close to its command a wheel has to be to count as up to speed. */
    private static final double WHEELS_UP = 0.95;

    private static final String[] BALL_STAGE = {"INTAKE", "RAMP"};

    /**
     * One keyboard shortcut: what it is called, what it does, and what to draw.
     *
     * <p>The window is the only documentation anyone reads while driving, so the
     * handler and the on-screen legend are driven from the same list. A binding
     * that does something therefore cannot fail to appear on screen - it is not
     * a rule to remember, there is simply nowhere to write one that does not.
     */
    private static final class Shortcut {
        final int keyCode;
        final String cap;
        final String label;
        final Runnable action;

        Shortcut(int keyCode, String cap, String label, Runnable action) {
            this.keyCode = keyCode;
            this.cap = cap;
            this.label = label;
            this.action = action;
        }
    }

    private final List<Shortcut> shortcuts = Arrays.asList(
            new Shortcut(KeyEvent.VK_S, "S", "start", () -> startRequested = true),
            new Shortcut(KeyEvent.VK_R, "R", "restart", () -> resetRequested = true),
            new Shortcut(KeyEvent.VK_ESCAPE, "Esc", "quit", () -> open = false));

    private static final Color BG = new Color(0x1E1F22);
    private static final Color PANEL = new Color(0x2B2D30);
    private static final Color EDGE = new Color(0x43464B);
    private static final Color GRID = new Color(0x35383C);
    private static final Color TEXT = new Color(0xD6D6D6);
    private static final Color DIM = new Color(0x8A8D91);
    private static final Color FWD = new Color(0x4CC38A);
    private static final Color REV = new Color(0xE0625A);
    private static final Color OFF = new Color(0x5A5D62);
    private static final Color ROBOT = new Color(0x4C9AFF);
    private static final Color ACCENT = new Color(0xE0A83C);
    private static final Color WARN = new Color(0xE0625A);

    private static final Font MONO = new Font(Font.MONOSPACED, Font.PLAIN, 12);
    private static final Font MONO_BOLD = new Font(Font.MONOSPACED, Font.BOLD, 12);
    private static final Font LABEL = new Font(Font.SANS_SERIF, Font.BOLD, 11);
    private static final Font TITLE = new Font(Font.SANS_SERIF, Font.BOLD, 13);

    /** One frame of state, swapped in whole so the painter never sees a tear. */
    private static final class Snapshot {
        double x, y, heading;
        double forward, strafe, turn;
        boolean againstWall;
        double fl, fr, rl, rr;
        double intake, ramp;
        double flywheel, wheelSpeed;
        boolean wheelsUp;
        double rpmL, rpmCommanded;
        boolean hasBall;
        double ballProgress;
        int ballStage = -1;
        double goalBearing;
        boolean firing;
        double flow;
        String intakeState = "IDLE";
        String launchState = "IDLE";
        String phase = "";
        boolean crashed;
        List<Shot> shots = new ArrayList<Shot>();
        GamePadReadings pad = new GamePadReadings();
        List<String> telemetry = new ArrayList<String>();
        Deque<Point2D.Double> trail = new ArrayDeque<Point2D.Double>();
    }

    private volatile Snapshot snapshot = new Snapshot();
    private volatile String inputStatus = "";
    private volatile boolean open = true;
    private volatile boolean resetRequested;
    private volatile boolean startRequested;

    private final Deque<Point2D.Double> trail = new ArrayDeque<Point2D.Double>();
    private final List<String> telemetry = new ArrayList<String>();
    private final List<Shot> shots = new ArrayList<Shot>();
    private double flowPhase;
    private long lastNanos;

    private JFrame frame;
    private Panel panel;

    /** @return false when there is no display to draw on, such as on a build server. */
    public boolean open() {
        if (GraphicsEnvironment.isHeadless()) {
            System.err.println("No display available, so --gui cannot start. Run without it.");
            return false;
        }
        try {
            Runnable createWindow = new Runnable() {
                public void run() {
                    panel = new Panel();
                    frame = new JFrame("gobilda-experiment - teleop harness");
                    frame.setDefaultCloseOperation(JFrame.DO_NOTHING_ON_CLOSE);
                    frame.setContentPane(panel);
                    frame.pack();
                    frame.setLocationRelativeTo(null);
                    frame.addWindowListener(new WindowAdapter() {
                        @Override public void windowClosed(WindowEvent e) { open = false; }
                        @Override public void windowClosing(WindowEvent e) {
                            // Let the harness loop dispose the window on the
                            // macOS first thread instead of from Swing's EDT.
                            open = false;
                        }
                    });
                    panel.setFocusable(true);
                    panel.addKeyListener(new KeyAdapter() {
                        @Override public void keyPressed(KeyEvent e) {
                            for (Shortcut shortcut : shortcuts) {
                                if (e.getKeyCode() == shortcut.keyCode) shortcut.action.run();
                            }
                        }
                    });
                    frame.setVisible(true);
                    String osName = System.getProperty("os.name");
                    if (osName != null && osName.startsWith("Mac")) {
                        frame.setAlwaysOnTop(true);
                    }
                    frame.toFront();
                    frame.requestFocus();
                    panel.requestFocusInWindow();
                }
            };
            String osName = System.getProperty("os.name", "");
            if (osName != null && osName.startsWith("Mac")) {
                // With -XstartOnFirstThread, initializing AWT through
                // invokeAndWait can deadlock the macOS native event loop.
                createWindow.run();
            } else {
                SwingUtilities.invokeAndWait(createWindow);
            }
        } catch (Exception e) {
            System.err.println("Could not open the window: " + e);
            return false;
        }
        return true;
    }

    public boolean isOpen() {
        return open;
    }

    /** A line drawn on the gamepad panel, for trouble with the input itself. */
    public void setInputStatus(String status) {
        inputStatus = status == null ? "" : status;
    }

    public void close() {
        open = false;
        if (frame == null) return;
        String osName = System.getProperty("os.name", "");
        if (osName != null && osName.startsWith("Mac")) {
            // GLFW and AppKit operations both belong on the first thread.
            frame.dispose();
        } else {
            SwingUtilities.invokeLater(new Runnable() {
                public void run() { frame.dispose(); }
            });
        }
    }

    /** True once, if the operator asked for S: the Driver Station's START. */
    public boolean consumeStartRequest() {
        if (!startRequested) return false;
        startRequested = false;
        return true;
    }

    /** True once, if the operator asked to restart: back to INIT, robot re-centred. */
    public boolean consumeResetRequest() {
        if (!resetRequested) return false;
        resetRequested = false;
        return true;
    }

    /** Called from the harness loop; safe to call off the event thread. */
    public void update(GamePadReadings pad, DesktopRobot robot, RobotSim sim,
                       BallPath ball, List<String> newTelemetry, OpModeRunner runner) {
        Snapshot s = new Snapshot();
        if (runner != null) {
            s.phase = GamepadHarness.phaseLine(runner);
            s.crashed = runner.failure() != null;
        }
        s.x = sim.xInches;
        s.y = sim.yInches;
        s.heading = sim.headingDeg;
        s.forward = sim.forwardCmd;
        s.strafe = sim.strafeCmd;
        s.turn = sim.turnCmd;
        s.againstWall = sim.againstWall;
        s.fl = robot.frontLeft.power;
        s.fr = robot.frontRight.power;
        s.rl = robot.rearLeft.power;
        s.rr = robot.rearRight.power;
        s.intake = robot.intakeFlow();
        s.ramp = robot.rampFlow();
        s.flywheel = robot.flywheelCommand();
        s.wheelSpeed = robot.flywheelSpeed();
        s.wheelsUp = s.flywheel > IDLE && s.wheelSpeed >= s.flywheel * WHEELS_UP;
        s.rpmL = robot.leftWheelRPM();
        s.rpmCommanded = robot.commandedWheelRPM();
        s.goalBearing = sim.bearingToGoal();
        s.firing = s.ramp > IDLE;
        s.pad = pad;

        long nowNanos = System.nanoTime();
        double dt = lastNanos == 0 ? 0 : Math.min(0.1, (nowNanos - lastNanos) / 1e9);
        lastNanos = nowNanos;
        flowPhase += dt;
        s.flow = flowPhase;

        // State in words. A driver behind the field has no time to interpret
        // four signed decimals.
        s.intakeState = s.intake > IDLE ? "INTAKING" : s.intake < -IDLE ? "EJECTING" : "IDLE";
        if (s.firing) {
            s.launchState = "FEEDING";
        } else if (s.flywheel < -IDLE) {
            s.launchState = "REVERSING";
        } else if (s.flywheel > IDLE) {
            s.launchState = s.wheelsUp ? "READY" : "SPINNING UP";
        } else {
            s.launchState = "IDLE";
        }

        s.hasBall = ball.hasBall;
        s.ballProgress = ball.progress;
        s.ballStage = ball.stage();

        // A shot is a ball leaving, not a ramp turning. Pressing fire with an
        // empty path now draws nothing, which is what the robot would do.
        if (ball.takeShot()) {
            shots.add(new Shot(sim.xInches, sim.yInches, sim.headingDeg));
        }
        for (int i = shots.size() - 1; i >= 0; i--) {
            shots.get(i).age += dt;
            if (shots.get(i).age > SHOT_FADE_SECONDS) shots.remove(i);
        }
        s.shots = new ArrayList<Shot>(shots);

        Point2D.Double last = trail.peekLast();
        if (last == null || Math.hypot(last.x - sim.xInches, last.y - sim.yInches) > 0.6) {
            trail.addLast(new Point2D.Double(sim.xInches, sim.yInches));
            while (trail.size() > TRAIL_POINTS) trail.removeFirst();
        }
        s.trail = new ArrayDeque<Point2D.Double>(trail);

        if (newTelemetry != null && !newTelemetry.isEmpty()) {
            // The Driver Station replaces what it shows each update rather than
            // scrolling a log, so keep the newest frame instead of appending.
            telemetry.clear();
            telemetry.addAll(newTelemetry);
            while (telemetry.size() > TELEMETRY_LINES) telemetry.remove(0);
        }
        s.telemetry = new ArrayList<String>(telemetry);

        snapshot = s;
        if (panel != null) panel.repaint();
    }

    public void clearTrail() {
        trail.clear();
    }

    /**
     * Test seam: draws one frame into any Graphics2D, with no window involved.
     *
     * <p>A field view that throws halfway through a repaint shows up as a blank
     * or half-drawn panel and is easy to blame on the robot code, so the paint
     * path is worth covering headlessly like anything else.
     */
    void renderOffscreen(java.awt.Graphics2D g, int width, int height) {
        Panel p = new Panel();
        p.setSize(width, height);
        p.render(g);
    }

    // ------------------------------------------------------------------- drawing

    private final class Panel extends JPanel {

        Panel() {
            setPreferredSize(new Dimension(1240, 660));
            setBackground(BG);
        }

        @Override
        protected void paintComponent(Graphics g0) {
            super.paintComponent(g0);
            render((Graphics2D) g0);
        }

        void render(Graphics2D g) {
            g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
            g.setRenderingHint(RenderingHints.KEY_STROKE_CONTROL, RenderingHints.VALUE_STROKE_PURE);
            g.setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING,
                    RenderingHints.VALUE_TEXT_ANTIALIAS_ON);

            Snapshot s = snapshot;
            int w = getWidth();
            int h = getHeight();
            int m = 14;
            int statusH = 24;

            int fieldSize = Math.min(h - 2 * m - statusH, (int) ((w - 3 * m) * 0.50));
            int fx = m;
            int fy = m;
            int rx = m + fieldSize + m;
            int rw = w - rx - m;

            drawField(g, s, fx, fy, fieldSize);

            int y = m;
            int ballH = 150;
            drawBallPath(g, s, rx, y, rw, ballH);
            y += ballH + m;
            int padH = 196;
            drawGamepad(g, s, rx, y, rw, padH);
            y += padH + m;
            int driveH = 92;
            drawDrive(g, s, rx, y, rw, driveH);
            y += driveH + m;
            drawTelemetry(g, s, rx, y, rw, h - y - m - statusH);

            drawStatus(g, s, m, h - statusH, w - 2 * m, statusH);
        }

        private void panel(Graphics2D g, int x, int y, int w, int h, String title) {
            g.setColor(PANEL);
            g.fillRoundRect(x, y, w, h, 8, 8);
            g.setColor(EDGE);
            g.drawRoundRect(x, y, w, h, 8, 8);
            if (title != null) {
                g.setFont(TITLE);
                g.setColor(DIM);
                g.drawString(title, x + 10, y + 16);
            }
        }

        // -------------------------------------------------------------- the field

        private void drawField(Graphics2D g, Snapshot s, int x, int y, int size) {
            panel(g, x, y, size, size, null);

            double scale = size / RobotSim.FIELD_INCHES;

            // A shot ray, or a robot pushed to the edge, would otherwise draw
            // straight over the panels next to it.
            Shape savedClip = g.getClip();
            g.setClip(x, y, size + 1, size + 1);

            // Foam tiles, 24 inches each.
            g.setColor(GRID);
            g.setStroke(new BasicStroke(1f));
            for (int i = 1; i < 6; i++) {
                int p = (int) Math.round(i * 24 * scale);
                g.drawLine(x + p, y + 1, x + p, y + size - 1);
                g.drawLine(x + 1, y + p, x + size - 1, y + p);
            }

            // Trail, oldest faintest.
            int n = s.trail.size();
            int i = 0;
            Point2D.Double prev = null;
            g.setStroke(new BasicStroke(2f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
            for (Point2D.Double p : s.trail) {
                if (prev != null) {
                    int alpha = 20 + (int) (150.0 * i / Math.max(1, n));
                    g.setColor(new Color(ROBOT.getRed(), ROBOT.getGreen(), ROBOT.getBlue(), alpha));
                    g.drawLine(sx(x, prev.x, scale), sy(y, size, prev.y, scale),
                            sx(x, p.x, scale), sy(y, size, p.y, scale));
                }
                prev = p;
                i++;
            }

            // Shots: where a ball left, along the heading at that instant. A
            // marker of an event, not a ball trajectory.
            for (Shot shot : s.shots) {
                double fade = 1 - shot.age / SHOT_FADE_SECONDS;
                int alpha = (int) (210 * Math.max(0, fade));
                double b = Math.toRadians(shot.bearing);
                double len = 34 * scale;
                int x0 = sx(x, shot.x, scale);
                int y0 = sy(y, size, shot.y, scale);
                int x1 = x0 + (int) (Math.sin(b) * len);
                int y1 = y0 - (int) (Math.cos(b) * len);
                g.setColor(new Color(ACCENT.getRed(), ACCENT.getGreen(), ACCENT.getBlue(), alpha));
                g.setStroke(new BasicStroke(2f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
                g.drawLine(x0, y0, x1, y1);
                g.fillOval(x1 - 4, y1 - 4, 8, 8);
            }

            // The goal. The launcher is fixed, so aiming is steering the robot.
            int gx = sx(x, RobotSim.GOAL_X_INCHES, scale);
            int gy = sy(y, size, RobotSim.GOAL_Y_INCHES, scale);
            g.setColor(ACCENT);
            g.setStroke(new BasicStroke(2f));
            g.draw(new Ellipse2D.Double(gx - 9, gy - 9, 18, 18));
            g.drawLine(gx - 3, gy, gx + 3, gy);
            g.drawLine(gx, gy - 3, gx, gy + 3);
            g.setFont(LABEL);
            g.setColor(DIM);
            g.drawString("GOAL", gx + 13, gy + 4);

            // The robot itself.
            int cx = sx(x, s.x, scale);
            int cy = sy(y, size, s.y, scale);
            int side = (int) (RobotSim.ROBOT_INCHES * scale);

            AffineTransform saved = g.getTransform();
            g.translate(cx, cy);
            g.rotate(Math.toRadians(s.heading));

            // Where a shot would go. Drawn inside the rotated frame, so it
            // follows the chassis for free.
            g.setColor(ACCENT);
            g.setStroke(new BasicStroke(3f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
            g.drawLine(0, 0, 0, -(int) (side * 0.95));
            g.fillOval(-4, -(int) (side * 0.95) - 4, 8, 8);

            g.setColor(new Color(ROBOT.getRed(), ROBOT.getGreen(), ROBOT.getBlue(), 60));
            g.fillRect(-side / 2, -side / 2, side, side);
            g.setColor(ROBOT);
            g.setStroke(new BasicStroke(2f));
            g.drawRect(-side / 2, -side / 2, side, side);

            /*
             * The four wheels, where they sit on the chassis.
             *
             * The right-hand motors are reversed in the robot configuration
             * rather than in code, so a command of (+,-,+,-) is the robot
             * driving straight forward. Undo that mirror here, exactly as
             * RobotSim does, or the right wheels would be drawn spinning
             * backwards while the robot drives forward. DrivetrainMotorTest
             * pins the convention down; the DRIVE panel shows the raw numbers.
             */
            double[] spin = {s.fl, -s.fr, s.rl, -s.rr};
            double[] wheelX = {-side / 2.0, side / 2.0, -side / 2.0, side / 2.0};
            double[] wheelY = {-side * 0.27, -side * 0.27, side * 0.27, side * 0.27};
            // Mecanum rollers make an X seen from above: FL and RR lean one
            // way, FR and RL the other. Drawn so this is a picture of THIS
            // drivetrain rather than of any four wheels.
            boolean[] rollerLeansRight = {true, false, false, true};
            for (int k = 0; k < 4; k++) {
                drawWheel(g, wheelX[k], wheelY[k], side * 0.19, side * 0.34,
                        spin[k], rollerLeansRight[k]);
            }

            // Which end is the front.
            Path2D.Double nose = new Path2D.Double();
            nose.moveTo(0, -side / 2.0 - 1);
            nose.lineTo(-side * 0.22, -side * 0.18);
            nose.lineTo(side * 0.22, -side * 0.18);
            nose.closePath();
            g.fill(nose);

            g.setTransform(saved);
            g.setClip(savedClip);

            g.setColor(EDGE);
            g.setStroke(new BasicStroke(1f));
            g.drawRoundRect(x, y, size, size, 8, 8);

            g.setFont(LABEL);
            g.setColor(DIM);
            g.drawString("FIELD  12 ft, one square is one tile   -   wheels show which way each one turns",
                    x + 10, y + size - 10);
            // What the OpMode is doing, as the Driver Station would say it.
            int below = y + 38;
            if (!s.phase.isEmpty()) {
                // A crash puts its detail on a second line, which is the part
                // worth reading and too long to share the first.
                String[] parts = s.phase.split(" - ", 2);
                g.setFont(TITLE);
                g.setColor(s.crashed ? WARN : TEXT);
                g.drawString(parts[0], x + 10, y + 20);
                if (parts.length > 1) {
                    g.setFont(LABEL);
                    g.drawString(parts[1], x + 10, y + 36);
                    below = y + 54;
                }
                g.setFont(LABEL);
            }
            if (s.againstWall) {
                g.setColor(WARN);
                g.drawString("AGAINST WALL", x + 10, below);
            }

            g.setFont(LABEL);
            g.setColor(ACCENT);
            g.drawString(String.format(Locale.ROOT,
                    "GOAL %+.0f deg off the nose  -  the launcher is fixed, so steer to aim",
                    s.goalBearing), x + 10, y + size - 26);
        }

        /**
         * One wheel, seen from above, filled out from its midline by how hard
         * it is being driven: up the screen for forward, down for reverse, and
         * empty when it is stopped.
         *
         * @param forward which way this wheel actually turns, with the
         *     configuration's right-hand reversal already undone.
         */
        private void drawWheel(Graphics2D g, double cx, double cy, double w, double h,
                               double forward, boolean rollerLeansRight) {
            double p = clamp(forward);
            Color c = p > IDLE ? FWD : p < -IDLE ? REV : OFF;

            RoundRectangle2D.Double body = new RoundRectangle2D.Double(
                    cx - w / 2, cy - h / 2, w, h, w * 0.6, w * 0.6);

            g.setColor(new Color(BG.getRed(), BG.getGreen(), BG.getBlue(), 225));
            g.fill(body);

            if (Math.abs(p) > IDLE) {
                double fill = Math.abs(p) * (h / 2);
                Shape outside = g.getClip();
                g.clip(body);
                g.setColor(new Color(c.getRed(), c.getGreen(), c.getBlue(), 170));
                g.fill(new RoundRectangle2D.Double(
                        cx - w / 2, p > 0 ? cy - fill : cy, w, fill, 1, 1));
                g.setClip(outside);
            }

            g.setColor(c);
            g.setStroke(new BasicStroke(1.5f));
            g.draw(body);

            // The roller angle. A stopped wheel still shows which way it rolls.
            double r = w * 0.40;
            double dx = rollerLeansRight ? r : -r;
            g.setColor(new Color(c.getRed(), c.getGreen(), c.getBlue(), 130));
            g.setStroke(new BasicStroke(1f));
            g.draw(new Line2D.Double(cx - dx, cy - r, cx + dx, cy + r));
        }

        private int sx(int originX, double inches, double scale) {
            return originX + (int) Math.round(inches * scale);
        }

        private int sy(int originY, int size, double inches, double scale) {
            return originY + size - (int) Math.round(inches * scale);
        }

        // ----------------------------------------------------------- the ball path

        private void drawBallPath(Graphics2D g, Snapshot s, int x, int y, int w, int h) {
            boolean starved = s.firing && !s.wheelsUp;

            panel(g, x, y, w, h, "BALL PATH   intake to launcher");

            int stageY = y + 30;
            int stageH = 44;
            int gap = 26;
            int count = 3;
            int stageW = (w - 20 - gap * (count - 1)) / count;

            String[] names = {"INTAKE", "RAMP", "FLYWHEELS"};
            double[] values = {s.intake, s.ramp, s.flywheel};

            for (int i = 0; i < count; i++) {
                int stageX = x + 10 + i * (stageW + gap);
                drawStage(g, stageX, stageY, stageW, stageH, names[i], values[i]);
                if (i < count - 1) {
                    drawFlow(g, stageX + stageW, stageY + stageH / 2, gap, values[i], s.flow);
                }
            }

            /*
              * The ball itself, riding the strip. A stage lighting up says a
              * motor is turning; this says whether anything is going to come
              * out of it.
              */
             if (s.hasBall) {
                 int trackX = x + 10;
                 // The path ends where the wheels begin, so a ball they have
                 // not taken is drawn waiting at them rather than already through.
                 int trackW = 2 * (stageW + gap);
                 int bx = trackX + (int) (Math.max(0, Math.min(1, s.ballProgress)) * trackW);
                 int by = stageY + stageH / 2;
                 g.setColor(new Color(ACCENT.getRed(), ACCENT.getGreen(), ACCENT.getBlue(), 70));
                 g.fill(new Ellipse2D.Double(bx - 11, by - 11, 22, 22));
                 g.setColor(ACCENT);
                 g.setStroke(new BasicStroke(2f));
                 g.draw(new Ellipse2D.Double(bx - 8, by - 8, 16, 16));
             }

            int msgY = stageY + stageH + 22;
            g.setFont(MONO_BOLD);
            if (starved) {
                g.setColor(WARN);
                g.drawString("FEEDING BEFORE THE WHEELS ARE UP", x + 10, msgY);
            } else {
                g.setColor(s.intake > IDLE ? FWD : s.intake < -IDLE ? REV : DIM);
                g.drawString("INTAKE " + s.intakeState, x + 10, msgY);
                g.setColor("FEEDING".equals(s.launchState) || "READY".equals(s.launchState) ? FWD
                        : "REVERSING".equals(s.launchState) ? REV : DIM);
                g.drawString("LAUNCHER " + s.launchState, x + 200, msgY);
                if (s.firing && !s.hasBall) {
                    g.setColor(WARN);
                    g.drawString("NO BALL", x + 400, msgY);
                } else if (s.hasBall) {
                    g.setColor(DIM);
                    g.drawString("BALL AT " + BALL_STAGE[s.ballStage], x + 400, msgY);
                }
            }

            /*
             * Where the modelled wheels are, against what they were told. There
             * is no interlock on this robot: the ramp feeds as soon as it is
             * asked, and nothing asks the wheels. Which is why the bar goes red
             * while feeding, and the reason to model this at all.
             */
            int barY = msgY + 12;
            int barW = w - 20;
            g.setColor(EDGE);
            g.setStroke(new BasicStroke(1f));
            g.drawRect(x + 10, barY, barW, 8);
            double reached = s.rpmCommanded == 0 ? 0
                    : Math.max(0, Math.min(1, s.rpmL / s.rpmCommanded));
            int fill = (int) (reached * (barW - 1));
            g.setColor(starved ? WARN : s.wheelsUp ? FWD : ACCENT);
            g.fillRect(x + 11, barY + 1, fill, 7);

            g.setFont(MONO);
            g.setColor(DIM);
            String state = s.flywheel < -IDLE ? "wheels reversing"
                    : s.wheelsUp ? "wheels up to speed"
                    : s.flywheel > IDLE ? "wheels spinning up"
                    : "no wheel command";
            g.drawString(s.rpmCommanded == 0 ? state
                            : String.format(Locale.ROOT, "%s   |   %.0f of %.0f rpm modelled",
                                    state, s.rpmL, s.rpmCommanded),
                    x + 10, barY + 24);
        }

        /**
         * The keyboard shortcuts, as key caps, plus the one that is on the pad
         * rather than the keyboard. Laid out left to right from the same list
         * the key handler dispatches on, so a new binding appears here whether
         * or not anyone remembered it was supposed to.
         */
        private void drawKeyLegend(Graphics2D g, int x, int y, int w) {
            g.setFont(MONO);
            int capH = 16;
            int cursor = x;

            for (Shortcut shortcut : shortcuts) {
                int capW = Math.max(18, g.getFontMetrics().stringWidth(shortcut.cap) + 10);
                g.setColor(EDGE);
                g.setStroke(new BasicStroke(1f));
                g.fill(new RoundRectangle2D.Double(cursor, y, capW, capH, 4, 4));
                g.setColor(TEXT);
                g.drawString(shortcut.cap,
                        cursor + (capW - g.getFontMetrics().stringWidth(shortcut.cap)) / 2,
                        y + capH - 4);
                cursor += capW + 5;

                g.setColor(DIM);
                g.drawString(shortcut.label, cursor, y + capH - 4);
                cursor += g.getFontMetrics().stringWidth(shortcut.label) + 16;
            }

            // Not keyboard shortcuts, and worth saying so: they are the pad's.
            String pad = "START runs  BACK+START quits";
            int padW = g.getFontMetrics().stringWidth(pad);
            if (cursor + padW <= x + w) {
                g.setColor(DIM);
                g.drawString(pad, x + w - padW, y + capH - 4);
            }
        }

        private void drawStage(Graphics2D g, int x, int y, int w, int h, String name, double value) {
            Color c = value > IDLE ? FWD : value < -IDLE ? REV : OFF;
            g.setColor(new Color(c.getRed(), c.getGreen(), c.getBlue(), 42));
            g.fillRoundRect(x, y, w, h, 6, 6);
            g.setColor(c);
            g.setStroke(new BasicStroke(1.5f));
            g.drawRoundRect(x, y, w, h, 6, 6);

            g.setFont(LABEL);
            g.drawString(name, x + 8, y + 17);
            g.setFont(MONO_BOLD);
            g.setColor(TEXT);
            g.drawString(String.format(Locale.ROOT, "%+.2f", value), x + 8, y + 34);
        }

        /**
         * Chevrons crawling between two stages. Direction and speed both come
         * from that stage's own commanded power, so which way a ball would be
         * travelling - and how hard - reads without looking at the numbers.
         */
        private void drawFlow(Graphics2D g, int x, int y, int width, double value, double phase) {
            Color c = value > IDLE ? FWD : value < -IDLE ? REV : OFF;
            g.setColor(c);
            g.setStroke(new BasicStroke(2f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
            int x1 = x + 5;
            int x2 = x + width - 5;
            if (Math.abs(value) <= IDLE) {
                g.drawLine(x1, y, x2, y);
                return;
            }
            boolean rightwards = value > 0;
            int span = x2 - x1;
            int step = 9;
            double travel = phase * value * 60;
            double offset = ((travel % step) + step) % step;
            for (int at = -step; at <= span + step; at += step) {
                int cx = x1 + (int) (at + offset);
                if (cx < x1 || cx > x2) continue;
                int tip = rightwards ? cx + 3 : cx - 3;
                int back = rightwards ? cx - 2 : cx + 2;
                g.drawLine(back, y - 4, tip, y);
                g.drawLine(back, y + 4, tip, y);
            }
        }

        // ------------------------------------------------------------- the gamepad

        private void drawGamepad(Graphics2D g, Snapshot s, int x, int y, int w, int h) {
            panel(g, x, y, w, h, "GAMEPAD   what the OpMode sees");
            GamePadReadings p = s.pad;

            int stickR = 34;
            int cy = y + 34 + stickR;
            drawStick(g, x + 24 + stickR, cy, stickR, p.leftStickX, p.leftStickY, "L");
            drawStick(g, x + 24 + stickR * 3 + 24, cy, stickR, p.rightStickX, p.rightStickY, "R");

            int barX = x + 24 + stickR * 4 + 64;
            drawTriggerBar(g, barX, y + 34, 18, stickR * 2, p.leftTrigger, "LT");
            drawTriggerBar(g, barX + 44, y + 34, 18, stickR * 2, p.rightTrigger, "RT");

            int bx = barX + 104;
            int by = y + 36;
            drawPill(g, bx, by, "LB", p.leftBumper);
            drawPill(g, bx + 40, by, "RB", p.rightBumper);
            drawPill(g, bx, by + 28, "A", p.aButton);
            drawPill(g, bx + 40, by + 28, "B", p.bButton);
            drawPill(g, bx, by + 56, "X", p.xButton);
            drawPill(g, bx + 40, by + 56, "Y", p.yButton);

            int dx = bx + 100;
            int dy = by + 26;
            drawPill(g, dx + 26, dy - 26, "U", p.dPadUp);
            drawPill(g, dx, dy, "L", p.dPadLeft);
            drawPill(g, dx + 52, dy, "R", p.dPadRight);
            drawPill(g, dx + 26, dy + 26, "D", p.dPadDown);

            g.setFont(MONO);
            String status = inputStatus;
            if (status.isEmpty()) {
                drawKeyLegend(g, x + 10, y + h - 19, w - 20);
            } else {
                g.setColor(WARN);
                g.drawString(status, x + 10, y + h - 10);
            }
        }

        private void drawStick(Graphics2D g, int cx, int cy, int r, double vx, double vy, String tag) {
            g.setColor(EDGE);
            g.setStroke(new BasicStroke(1.5f));
            Shape circle = new Ellipse2D.Double(cx - r, cy - r, r * 2, r * 2);
            g.draw(circle);
            g.setColor(GRID);
            g.drawLine(cx - r, cy, cx + r, cy);
            g.drawLine(cx, cy - r, cx, cy + r);

            // The SDK gamepad reports stick Y negative when pushed up, and
            // DrivetrainControls negates it. Use it as-is here so the dot on
            // screen goes up when the driver pushes up.
            int px = cx + (int) (clamp(vx) * r);
            int py = cy + (int) (clamp(vy) * r);
            g.setColor(ROBOT);
            g.fillOval(px - 5, py - 5, 10, 10);
            g.setFont(LABEL);
            g.setColor(DIM);
            g.drawString(tag, cx - r, cy + r + 14);
        }

        private void drawTriggerBar(Graphics2D g, int x, int y, int w, int h, double v, String tag) {
            g.setColor(EDGE);
            g.drawRect(x, y, w, h);
            int fill = (int) (Math.max(0, Math.min(1, v)) * (h - 2));
            g.setColor(v > IDLE ? ROBOT : OFF);
            g.fillRect(x + 1, y + h - 1 - fill, w - 1, fill);
            g.setFont(LABEL);
            g.setColor(DIM);
            g.drawString(tag, x - 2, y + h + 14);
        }

        private void drawPill(Graphics2D g, int x, int y, String label, boolean on) {
            Shape r = new RoundRectangle2D.Double(x, y, 32, 22, 8, 8);
            g.setColor(on ? ROBOT : PANEL);
            g.fill(r);
            g.setColor(on ? ROBOT : EDGE);
            g.draw(r);
            g.setFont(LABEL);
            g.setColor(on ? Color.BLACK : DIM);
            int tw = g.getFontMetrics().stringWidth(label);
            g.drawString(label, x + 16 - tw / 2, y + 15);
        }

        // --------------------------------------------------------------- the drive

        private void drawDrive(Graphics2D g, Snapshot s, int x, int y, int w, int h) {
            panel(g, x, y, w, h, "DRIVE   recorded motor powers");
            g.setFont(MONO);
            g.setColor(TEXT);
            g.drawString(String.format(Locale.ROOT,
                    "FL %+.2f   FR %+.2f   RL %+.2f   RR %+.2f",
                    s.fl, s.fr, s.rl, s.rr), x + 10, y + 42);
            g.setColor(DIM);
            g.drawString(String.format(Locale.ROOT,
                    "recovered   forward %+.2f   strafe %+.2f   turn %+.2f",
                    s.forward, s.strafe, s.turn), x + 10, y + 62);
        }

        // ----------------------------------------------------------- the telemetry

        private void drawTelemetry(Graphics2D g, Snapshot s, int x, int y, int w, int h) {
            if (h < 40) return;
            panel(g, x, y, w, h, "TELEMETRY   as the Driver Station would show it");
            g.setFont(MONO);
            g.setColor(TEXT);
            int line = y + 36;
            for (String t : s.telemetry) {
                if (line > y + h - 6) break;
                g.drawString(t, x + 10, line);
                line += 15;
            }
        }

        private void drawStatus(Graphics2D g, Snapshot s, int x, int y, int w, int h) {
            g.setFont(MONO);
            g.setColor(DIM);
            g.drawString(String.format(Locale.ROOT,
                    "x %6.1f in   y %6.1f in   heading %+7.1f deg       "
                            + "dead reckoning only - no mass, no slip, no trajectory",
                    s.x, s.y, s.heading), x + 2, y + 16);
        }

        private double clamp(double v) {
            return Math.max(-1, Math.min(1, v));
        }
    }
}

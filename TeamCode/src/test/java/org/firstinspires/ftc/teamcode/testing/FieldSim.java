package org.firstinspires.ftc.teamcode.testing;

import org.firstinspires.ftc.teamcode.testing.BiobuzzField.Alliance;
import org.firstinspires.ftc.teamcode.testing.BiobuzzField.BallType;
import org.firstinspires.ftc.teamcode.testing.BiobuzzField.End;
import org.firstinspires.ftc.teamcode.testing.BiobuzzField.StartPose;
import org.firstinspires.ftc.teamcode.testing.BiobuzzField.StartSpot;
import org.firstinspires.ftc.teamcode.testing.BiobuzzField.V3;

import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Deque;
import java.util.EnumMap;
import java.util.List;
import java.util.Random;

/**
 * What moves on the BIOBUZZ field: the SCORING ELEMENTS, what the robot holds,
 * and which way each HIVE is tipped. {@link BiobuzzField} is what does not.
 *
 * <p><b>Still a picture, not physics.</b> Balls are discs that roll, slow
 * down, bounce off walls, the frame's base bars, the FLOWERS, each other and
 * the robot. The robot is a square that the walls, bars and FLOWERS stop.
 * Nothing has mass except for deciding when a HIVE tips. A LAUNCH has no
 * trajectory: it either goes into an upward CELL - if the robot is pointing at
 * the CELL's opening, within the opening's width, and the flywheels are up to
 * speed - or lands on the floor along the heading, further the faster the
 * wheels. Those launch numbers are guesses about a robot nobody measured; they
 * are constants below, there to be changed.
 *
 * <p>Not modelled: human players entering NECTAR, scoring, FLOWER ownership,
 * other robots, and anything to do with height except the HIVE (balls on the
 * floor never reach the CELLS).
 */
public final class FieldSim {

    /** Below this an intake command is not running. */
    private static final double IDLE = 0.02;

    /** How wide the intake mouth is, at the back of the chassis. A guess. */
    public static final double MOUTH_HALF_WIDTH = 6.0;

    /** Flywheel speed (fraction of free speed) below which a LAUNCH cannot reach a CELL. A guess. */
    public static final double MIN_SCORING_SPEED = 0.35;
    /** Horizontal distances, launcher to CELL opening, a LAUNCH can score from. Guesses. */
    public static final double MIN_SCORING_RANGE = 12, MAX_SCORING_RANGE = 132;
    /** How far a missed LAUNCH lands, per unit of flywheel speed. A guess. */
    public static final double MISS_RANGE_PER_SPEED = 150;

    /** How fast a rolling ball slows, inches per second squared. */
    private static final double ROLL_DECEL = 40;
    private static final double WALL_BOUNCE = 0.4, BALL_BOUNCE = 0.6, ROBOT_BOUNCE = 0.3;
    private static final double SUBSTEP = 0.01;

    /** A ball loose on the floor. */
    public static final class Ball {
        public final BallType type;
        public double x, y, vx, vy;

        Ball(BallType type, double x, double y) {
            this.type = type;
            this.x = x;
            this.y = y;
        }
    }

    /** One LAUNCH, for drawing: where it left, and where it went. */
    public static final class Shot {
        public final double x, y, landX, landY;
        /** "RED AUDIENCE CELL" if it went in, else null. */
        public final String cell;

        Shot(double x, double y, double landX, double landY, String cell) {
            this.x = x;
            this.y = y;
            this.landX = landX;
            this.landY = landY;
            this.cell = cell;
        }
    }

    private final List<Ball> balls = new ArrayList<Ball>();
    private final int[] flowerPollen = new int[BiobuzzField.FLOWERS.size()];
    private final EnumMap<Alliance, End> up = new EnumMap<Alliance, End>(Alliance.class);
    private final EnumMap<Alliance, EnumMap<End, List<BallType>>> cells =
            new EnumMap<Alliance, EnumMap<End, List<BallType>>>(Alliance.class);
    private final EnumMap<Alliance, Integer> tips = new EnumMap<Alliance, Integer>(Alliance.class);
    private final Deque<BallType> held = new ArrayDeque<BallType>();
    private final List<Shot> shots = new ArrayList<Shot>();
    private BallType inPath;
    private double flowerCooldown;
    private StartPose start = BiobuzzField.DEFAULT_START.pose();
    private StartSpot preset = BiobuzzField.DEFAULT_START;
    private Random random = new Random(1);

    public FieldSim() {
        reset(preset);
    }

    /** Stages the field afresh with the robot to start on {@code spot}. */
    public void reset(StartSpot spot) {
        preset = spot;
        reset(spot.pose());
    }

    /** Stages the field as Section 10.3.1 says, with the robot's pre-loads in it. */
    public void reset(StartPose pose) {
        start = pose;
        random = new Random(1);
        balls.clear();
        shots.clear();
        held.clear();
        inPath = null;
        flowerCooldown = 0;
        for (int i = 0; i < flowerPollen.length; i++) flowerPollen[i] = BiobuzzField.FLOWER_POLLEN;
        for (Alliance a : Alliance.values()) {
            up.put(a, BiobuzzField.upAtStart(a));
            tips.put(a, 0);
            EnumMap<End, List<BallType>> both = new EnumMap<End, List<BallType>>(End.class);
            for (End e : End.values()) both.put(e, new ArrayList<BallType>());
            for (int i = 0; i < BiobuzzField.CELL_NECTAR_AT_START; i++) {
                both.get(up.get(a)).add(BallType.nectar(a));
            }
            cells.put(a, both);
            for (Point2D.Double p : BiobuzzField.gardenPollen(a)) {
                balls.add(new Ball(BallType.POLLEN, p.x, p.y));
            }
        }
        for (int i = 0; i < BiobuzzField.PRELOAD_POLLEN; i++) held.add(BallType.POLLEN);
    }

    public StartPose start() {
        return start;
    }

    /** The ready-made spot after the last one used, for cycling through them. */
    public StartSpot nextPreset() {
        return preset.next();
    }

    /**
     * Moves where the robot starts without restaging the field. The pose must
     * be legal ({@link BiobuzzField#isLegalStart}); anything else is refused.
     */
    public void setStart(StartPose pose) {
        if (!BiobuzzField.isLegalStart(pose.alliance, pose.x, pose.y, pose.headingDeg)) {
            throw new IllegalArgumentException("not a legal start: " + pose.label);
        }
        start = pose;
    }

    /** Puts the robot on its start spot, stopped. */
    public void placeRobot(RobotSim sim) {
        sim.setStartingPose(start.x, start.y, start.headingDeg);
    }

    // ------------------------------------------------------- LEAVE and PARK

    /** Section 10.5.4 LEAVE: the robot is no longer contacting the perimeter wall. */
    public static boolean offWall(RobotSim sim) {
        double min = Double.MAX_VALUE, max = -Double.MAX_VALUE;
        double minY = Double.MAX_VALUE, maxY = -Double.MAX_VALUE;
        for (double[] c : BiobuzzField.robotCorners(sim.xInches, sim.yInches, sim.headingDeg)) {
            min = Math.min(min, c[0]);
            max = Math.max(max, c[0]);
            minY = Math.min(minY, c[1]);
            maxY = Math.max(maxY, c[1]);
        }
        double size = BiobuzzField.FIELD_INCHES, touch = 0.05;
        return min > touch && minY > touch && max < size - touch && maxY < size - touch;
    }

    /** Section 10.5.4 PARK: the robot is at least partially in its ALLIANCE's LOADING ZONE. */
    public boolean parked(RobotSim sim) {
        Rectangle2D.Double zone = start.alliance == Alliance.RED
                ? BiobuzzField.RED_LOADING_ZONE : BiobuzzField.BLUE_LOADING_ZONE;
        return obbOutOfRect(sim.xInches, sim.yInches, sim.headingDeg, RobotSim.ROBOT_INCHES / 2, zone) != null;
    }

    /** Test seam: a ball on the floor at (x, y). */
    Ball addBall(BallType type, double x, double y) {
        Ball b = new Ball(type, x, y);
        balls.add(b);
        return b;
    }

    /** Test seam: the robot holds exactly these, none in the path. */
    void hold(BallType... types) {
        held.clear();
        inPath = null;
        Collections.addAll(held, types);
    }

    // ------------------------------------------------------------ what it holds

    /** Balls the robot holds, including the one in the intake/ramp path. */
    public int carried() {
        return held.size() + (inPath != null ? 1 : 0);
    }

    /** For {@link BallPath}: hands the next held ball to the intake/ramp path. */
    public boolean takeForPath() {
        if (held.isEmpty()) return false;
        inPath = held.poll();
        return true;
    }

    /**
     * A ball left the flywheels. It goes into the upward CELL the robot is
     * pointing at, if any is within reach; otherwise it lands on the floor.
     */
    public Shot launch(RobotSim sim, double flywheelSpeed) {
        BallType type = inPath != null ? inPath : BallType.POLLEN;
        inPath = null;
        double h = Math.toRadians(sim.headingDeg);
        double fx = Math.sin(h), fy = Math.cos(h);
        double half = RobotSim.ROBOT_INCHES / 2;
        double ox = sim.xInches + fx * half, oy = sim.yInches + fy * half;

        Alliance hit = null;
        double nearest = Double.MAX_VALUE;
        V3 target = null;
        if (flywheelSpeed >= MIN_SCORING_SPEED) {
            for (Alliance a : Alliance.values()) {
                V3 o = BiobuzzField.opening(a, up.get(a), up.get(a));
                double along = (o.x - ox) * fx + (o.y - oy) * fy;
                double across = (o.x - ox) * fy - (o.y - oy) * fx;
                if (along >= MIN_SCORING_RANGE && along <= MAX_SCORING_RANGE
                        && Math.abs(across) <= BiobuzzField.CELL_WIDTH / 2 && along < nearest) {
                    nearest = along;
                    hit = a;
                    target = o;
                }
            }
        }

        Shot shot;
        if (hit != null) {
            End cell = up.get(hit);
            cells.get(hit).get(cell).add(type);
            shot = new Shot(ox, oy, target.x, target.y, hit + " " + cell + " CELL");
            checkTip(hit);
        } else {
            double range = Math.max(6, MISS_RANGE_PER_SPEED * flywheelSpeed);
            double r = type.radius;
            double lx = clamp(ox + fx * range, r, BiobuzzField.FIELD_INCHES - r);
            double ly = clamp(oy + fy * range, r, BiobuzzField.FIELD_INCHES - r);
            Ball landed = new Ball(type, lx, ly);
            landed.vx = fx * 20;
            landed.vy = fy * 20;
            balls.add(landed);
            shot = new Shot(ox, oy, lx, ly, null);
        }
        shots.add(shot);
        return shot;
    }

    /** A ball was spat back out of the intake: it drops behind the robot. */
    public void spit(RobotSim sim) {
        if (inPath == null) return;
        double h = Math.toRadians(sim.headingDeg);
        double fx = Math.sin(h), fy = Math.cos(h);
        double back = RobotSim.ROBOT_INCHES / 2 + inPath.radius + 0.5;
        Ball b = new Ball(inPath, sim.xInches - fx * back, sim.yInches - fy * back);
        b.vx = -fx * 15;
        b.vy = -fy * 15;
        balls.add(b);
        inPath = null;
    }

    // ------------------------------------------------------------------ HIVES

    private void checkTip(Alliance a) {
        End was = up.get(a);
        double weight = 0;
        for (BallType t : cells.get(a).get(was)) weight += t.weight;
        if (weight + 1e-9 < BiobuzzField.TIP_WEIGHT) return;

        End now = was.other();
        up.put(a, now);
        tips.put(a, tips.get(a) + 1);
        // The CELL that was up is now pointing down: what was in it falls out.
        V3 mouth = BiobuzzField.opening(a, was, now);
        List<BallType> spilled = cells.get(a).get(was);
        for (BallType t : spilled) {
            double angle = Math.toRadians(was.sign > 0 ? 0 : 180) + (random.nextDouble() - 0.5) * 2.0;
            double speed = 25 + random.nextDouble() * 25;
            Ball b = new Ball(t, mouth.x + (random.nextDouble() - 0.5) * 6, mouth.y + (random.nextDouble() - 0.5) * 6);
            b.vx = Math.sin(angle) * speed;
            b.vy = Math.cos(angle) * speed;
            balls.add(b);
        }
        spilled.clear();
    }

    /** Which end of a HIVE is up now. */
    public End up(Alliance a) {
        return up.get(a);
    }

    /** What is in a CELL. */
    public List<BallType> cell(Alliance a, End end) {
        return Collections.unmodifiableList(cells.get(a).get(end));
    }

    public int tips(Alliance a) {
        return tips.get(a);
    }

    /** The upward CELL nearest (x, y), for aiming at. */
    public Alliance nearestUpHive(double x, double y) {
        Alliance best = null;
        double bestDistance = Double.MAX_VALUE;
        for (Alliance a : Alliance.values()) {
            V3 o = BiobuzzField.opening(a, up.get(a), up.get(a));
            double d = Math.hypot(o.x - x, o.y - y);
            if (d < bestDistance) {
                bestDistance = d;
                best = a;
            }
        }
        return best;
    }

    public List<Ball> balls() {
        return Collections.unmodifiableList(balls);
    }

    public int flowerPollen(int flower) {
        return flowerPollen[flower];
    }

    /** LAUNCHES since the last call, oldest first. */
    public List<Shot> drainShots() {
        List<Shot> out = new ArrayList<Shot>(shots);
        shots.clear();
        return out;
    }

    // -------------------------------------------------------------- stepping

    /**
     * Moves the balls and keeps the robot out of the field elements, over
     * {@code dtSec}. Call after {@link RobotSim#step}.
     *
     * @param intake the intake's command, positive pulling in
     */
    public void step(RobotSim sim, double intake, double dtSec) {
        if (dtSec <= 0) return;
        int n = (int) Math.ceil(dtSec / SUBSTEP);
        double dt = dtSec / n;
        for (int i = 0; i < n; i++) {
            keepRobotOut(sim);
            moveBalls(sim, intake, dt);
            takeFromFlowers(sim, intake, dt);
        }
    }

    private void keepRobotOut(RobotSim sim) {
        double half = RobotSim.ROBOT_INCHES / 2;
        for (Rectangle2D.Double bar : BiobuzzField.FRAME_BARS) {
            double[] push = obbOutOfRect(sim.xInches, sim.yInches, sim.headingDeg, half, bar);
            if (push != null) moveRobot(sim, push[0], push[1]);
        }
        for (Point2D.Double f : BiobuzzField.FLOWERS) {
            double[] c = closestOnRobot(sim, f.x, f.y);
            double dx = c[0] - f.x, dy = c[1] - f.y;
            double d = Math.hypot(dx, dy);
            double r = BiobuzzField.FLOWER_RADIUS;
            if (d < 1e-9) {
                // The FLOWER's center is on or inside the robot: clear its bounding square.
                double[] push = obbOutOfRect(sim.xInches, sim.yInches, sim.headingDeg, half,
                        new Rectangle2D.Double(f.x - r, f.y - r, 2 * r, 2 * r));
                if (push != null) moveRobot(sim, push[0], push[1]);
            } else if (d < r) {
                moveRobot(sim, dx / d * (r - d), dy / d * (r - d));
            }
        }
    }

    private static void moveRobot(RobotSim sim, double dx, double dy) {
        sim.xInches += dx;
        sim.yInches += dy;
        double d = Math.hypot(dx, dy);
        if (d > 0) sim.blockedAlong(-dx / d, -dy / d);
    }

    /**
     * The smallest move that takes a square of half-width {@code half}, at
     * (cx, cy) turned {@code headingDeg}, out of {@code r}; null if apart.
     */
    static double[] obbOutOfRect(double cx, double cy, double headingDeg, double half, Rectangle2D.Double r) {
        double h = Math.toRadians(headingDeg);
        double[][] axes = {{1, 0}, {0, 1}, {Math.cos(h), -Math.sin(h)}, {Math.sin(h), Math.cos(h)}};
        double rcx = r.getCenterX(), rcy = r.getCenterY();
        double best = Double.MAX_VALUE;
        double[] bestAxis = null;
        for (double[] ax : axes) {
            double robotRadius = half * (Math.abs(ax[0] * Math.cos(h) - ax[1] * Math.sin(h))
                    + Math.abs(ax[0] * Math.sin(h) + ax[1] * Math.cos(h)));
            double rectRadius = r.width / 2 * Math.abs(ax[0]) + r.height / 2 * Math.abs(ax[1]);
            double between = (cx - rcx) * ax[0] + (cy - rcy) * ax[1];
            double overlap = robotRadius + rectRadius - Math.abs(between);
            if (overlap <= 0) return null;
            if (overlap < best) {
                best = overlap;
                double sign = between >= 0 ? 1 : -1;
                bestAxis = new double[] {ax[0] * sign, ax[1] * sign};
            }
        }
        return new double[] {bestAxis[0] * best, bestAxis[1] * best};
    }

    /** How far (px, py) is from the robot's square at (x, y), 0 if inside it. */
    static double distanceToRobot(double x, double y, double headingDeg, double px, double py) {
        double h = Math.toRadians(headingDeg);
        double half = RobotSim.ROBOT_INCHES / 2;
        double dx = px - x, dy = py - y;
        double lr = dx * Math.cos(h) - dy * Math.sin(h);
        double lf = dx * Math.sin(h) + dy * Math.cos(h);
        double ox = Math.max(0, Math.abs(lr) - half), oy = Math.max(0, Math.abs(lf) - half);
        return Math.hypot(ox, oy);
    }

    /** The point of the robot's square nearest (px, py), and its local (right, forward). */
    private static double[] closestOnRobot(RobotSim sim, double px, double py) {
        double h = Math.toRadians(sim.headingDeg);
        double rx = Math.cos(h), ry = -Math.sin(h), fx = Math.sin(h), fy = Math.cos(h);
        double half = RobotSim.ROBOT_INCHES / 2;
        double dx = px - sim.xInches, dy = py - sim.yInches;
        double lr = clamp(dx * rx + dy * ry, -half, half);
        double lf = clamp(dx * fx + dy * fy, -half, half);
        return new double[] {sim.xInches + rx * lr + fx * lf, sim.yInches + ry * lr + fy * lf, lr, lf};
    }

    private void moveBalls(RobotSim sim, double intake, double dt) {
        double size = BiobuzzField.FIELD_INCHES;
        for (int i = balls.size() - 1; i >= 0; i--) {
            Ball b = balls.get(i);
            double speed = Math.hypot(b.vx, b.vy);
            if (speed > 0) {
                double slower = Math.max(0, speed - ROLL_DECEL * dt);
                b.vx *= slower / speed;
                b.vy *= slower / speed;
            }
            b.x += b.vx * dt;
            b.y += b.vy * dt;

            double r = b.type.radius;
            if (b.x < r) { b.x = r; b.vx = Math.abs(b.vx) * WALL_BOUNCE; }
            if (b.x > size - r) { b.x = size - r; b.vx = -Math.abs(b.vx) * WALL_BOUNCE; }
            if (b.y < r) { b.y = r; b.vy = Math.abs(b.vy) * WALL_BOUNCE; }
            if (b.y > size - r) { b.y = size - r; b.vy = -Math.abs(b.vy) * WALL_BOUNCE; }

            for (Rectangle2D.Double bar : BiobuzzField.FRAME_BARS) {
                double cx = clamp(b.x, bar.x, bar.x + bar.width);
                double cy = clamp(b.y, bar.y, bar.y + bar.height);
                bounceOff(b, cx, cy, r);
            }
            for (Point2D.Double f : BiobuzzField.FLOWERS) {
                double dx = b.x - f.x, dy = b.y - f.y;
                double d = Math.hypot(dx, dy);
                if (d < BiobuzzField.FLOWER_RADIUS + r && d > 1e-9) {
                    bounceOff(b, f.x + dx / d * BiobuzzField.FLOWER_RADIUS,
                            f.y + dy / d * BiobuzzField.FLOWER_RADIUS, r);
                }
            }

            if (touchRobot(sim, b, intake)) balls.remove(i);
        }

        for (int i = 0; i < balls.size(); i++) {
            for (int j = i + 1; j < balls.size(); j++) collide(balls.get(i), balls.get(j));
        }
    }

    /** Pushes a ball off the point (cx, cy) of something fixed, if it overlaps. */
    private static void bounceOff(Ball b, double cx, double cy, double r) {
        double dx = b.x - cx, dy = b.y - cy;
        double d = Math.hypot(dx, dy);
        if (d >= r || d < 1e-9) return;
        double nx = dx / d, ny = dy / d;
        b.x = cx + nx * r;
        b.y = cy + ny * r;
        double vn = b.vx * nx + b.vy * ny;
        if (vn < 0) {
            b.vx -= (1 + WALL_BOUNCE) * vn * nx;
            b.vy -= (1 + WALL_BOUNCE) * vn * ny;
        }
    }

    /**
     * A ball touching the robot is either taken in, if it is at the intake
     * mouth with the intake running and room inside, or pushed away.
     *
     * @return true if it was taken in
     */
    private boolean touchRobot(RobotSim sim, Ball b, double intake) {
        double[] c = closestOnRobot(sim, b.x, b.y);
        double half = RobotSim.ROBOT_INCHES / 2;
        double dx = b.x - c[0], dy = b.y - c[1];
        double d = Math.hypot(dx, dy);
        double r = b.type.radius;
        boolean inside = d < 1e-9;
        if (!inside && d >= r) return false;

        double lr = c[2], lf = c[3];
        boolean atMouth = lf <= -half + r && Math.abs(lr) <= MOUTH_HALF_WIDTH;
        if (atMouth && intake > IDLE && carried() < BiobuzzField.ROBOT_CAPACITY) {
            held.add(b.type);
            return true;
        }

        double nx, ny, depth;
        if (inside) {
            // Centre inside the square: out through the nearest side.
            double h = Math.toRadians(sim.headingDeg);
            double rx = Math.cos(h), ry = -Math.sin(h), fx = Math.sin(h), fy = Math.cos(h);
            double ox = b.x - sim.xInches, oy = b.y - sim.yInches;
            double alongR = ox * rx + oy * ry, alongF = ox * fx + oy * fy;
            if (half - Math.abs(alongR) < half - Math.abs(alongF)) {
                double s = alongR >= 0 ? 1 : -1;
                nx = rx * s;
                ny = ry * s;
                depth = half - Math.abs(alongR) + r;
            } else {
                double s = alongF >= 0 ? 1 : -1;
                nx = fx * s;
                ny = fy * s;
                depth = half - Math.abs(alongF) + r;
            }
        } else {
            nx = dx / d;
            ny = dy / d;
            depth = r - d;
        }
        b.x += nx * depth;
        b.y += ny * depth;

        // The robot's surface velocity where it touches, clockwise turn included.
        double w = Math.toRadians(sim.omegaDps());
        double ox = c[0] - sim.xInches, oy = c[1] - sim.yInches;
        double sx = sim.vxIps() + w * oy, sy = sim.vyIps() - w * ox;
        double surface = sx * nx + sy * ny;
        double ball = b.vx * nx + b.vy * ny;
        if (ball < surface) {
            double kick = (surface - ball) * (1 + ROBOT_BOUNCE);
            b.vx += kick * nx;
            b.vy += kick * ny;
        }
        return false;
    }

    private static void collide(Ball a, Ball b) {
        double dx = b.x - a.x, dy = b.y - a.y;
        double d = Math.hypot(dx, dy);
        double min = a.type.radius + b.type.radius;
        if (d >= min || d < 1e-9) return;
        double nx = dx / d, ny = dy / d;
        double ma = a.type.weight, mb = b.type.weight;
        double overlap = min - d;
        a.x -= nx * overlap * mb / (ma + mb);
        a.y -= ny * overlap * mb / (ma + mb);
        b.x += nx * overlap * ma / (ma + mb);
        b.y += ny * overlap * ma / (ma + mb);
        double closing = (a.vx - b.vx) * nx + (a.vy - b.vy) * ny;
        if (closing <= 0) return;
        double j = (1 + BALL_BOUNCE) * closing / (1 / ma + 1 / mb);
        a.vx -= j / ma * nx;
        a.vy -= j / ma * ny;
        b.vx += j / mb * nx;
        b.vy += j / mb * ny;
    }

    /** Section 9.7: POLLEN can be pulled out of a FLOWER's bottom with the intake against it. */
    private void takeFromFlowers(RobotSim sim, double intake, double dt) {
        flowerCooldown = Math.max(0, flowerCooldown - dt);
        if (intake <= IDLE || flowerCooldown > 0 || carried() >= BiobuzzField.ROBOT_CAPACITY) return;
        double h = Math.toRadians(sim.headingDeg);
        double back = RobotSim.ROBOT_INCHES / 2;
        double mx = sim.xInches - Math.sin(h) * back, my = sim.yInches - Math.cos(h) * back;
        for (int i = 0; i < flowerPollen.length; i++) {
            Point2D.Double f = BiobuzzField.FLOWERS.get(i);
            if (flowerPollen[i] > 0 && Math.hypot(mx - f.x, my - f.y) <= BiobuzzField.FLOWER_RADIUS + 2.5) {
                flowerPollen[i]--;
                held.add(BallType.POLLEN);
                flowerCooldown = 0.4;
                return;
            }
        }
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}

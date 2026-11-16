package org.firstinspires.ftc.teamcode.testing;

import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * The BIOBUZZ (2026-27) field: its fixed geometry, where things start, and the
 * HIVE's shape at either tilt. {@link FieldSim} holds what moves.
 *
 * <p>Coordinates are inches with the origin in the corner of TILE A1: x runs
 * from the red ALLIANCE wall to the blue one, y from the audience wall to the
 * far ("scoring") wall, z up from the TILES. That is the view of the Competition
 * Manual's figures, with the audience at the bottom, and it is
 * {@link RobotSim}'s frame.
 *
 * <p>Sources, so each number can be checked: Competition Manual Section 9
 * ARENA (TU06), Section 10 Game Details and Section 11 Game Rules (TU09), the
 * Event Field Setup Guide (V1.0), and the SDK's own BIOBUZZ AprilTag library
 * ({@code AprilTagGameDatabase.getBioBuzzTagLibrary()}). The official
 * reference for anything not printed is the field CAD.
 */
public final class BiobuzzField {

    private BiobuzzField() {}

    /** Section 9.2: approximately 144 in. by 144 in., inside the walls. */
    public static final double FIELD_INCHES = 144.0;
    /** Section 9.2: 36 TILES, approximately 24 in. square. */
    public static final double TILE_INCHES = 24.0;
    public static final double CENTER = FIELD_INCHES / 2;

    public enum Alliance { RED, BLUE }

    /** Which end of its HIVE a CELL is on: the far wall's end or the audience's. */
    public enum End {
        SCORING(+1), AUDIENCE(-1);

        /** +1 if this end is towards larger y. */
        final int sign;

        End(int sign) {
            this.sign = sign;
        }

        public End other() {
            return this == SCORING ? AUDIENCE : SCORING;
        }
    }

    // ------------------------------------------------------------------ HIVE

    /** Figure 9-10: the HIVES are 25.5 in. apart, red on the red side. */
    public static double hiveX(Alliance alliance) {
        return alliance == Alliance.RED ? CENTER - 12.75 : CENTER + 12.75;
    }

    /** Section 9.6.1: pivot axis 43.95 in. above the TILES, at the field's center line. */
    public static final double PIVOT_HEIGHT = 43.95;
    /** Figure 9-10: a tipped HIVE sits 30 degrees off level. */
    public static final double TILT_DEG = 30.0;

    /** Figure 9-9: 42.91 in. end to end, CELLS 12.04 in. deep with 18.84 in. between. */
    public static final double CELL_INNER = 18.84 / 2, CELL_OUTER = 42.91 / 2;
    /** Section 9.6.2: the CELL opening is approximately 20 in. wide by 14 in. tall. */
    public static final double CELL_WIDTH = 20.0, CELL_HEIGHT = 14.0;

    /**
     * The CELL opening's center, which is the AprilTag Cluster's origin
     * (SDK README: "the Origin of each cluster is located in the center of the
     * Cell opening"). In the HIVE's own frame - along the HIVE from the pivot,
     * and up off its floor - it is at the CELL's outer end, 5.622 in. above the
     * tags (the SDK's cluster offsetZ). That puts an upward CELL's opening at
     * 59.55 in., the middle of Figure 9-10's 53.5 to 65.6 in.
     */
    public static final double OPENING_ALONG = CELL_OUTER, OPENING_UP = 5.622;

    /** SDK cluster members: 3.25 in. tags, this far from the origin across and down the sticker. */
    public static final double TAG_SIZE = 3.25;
    public static final double[] TAG_ACROSS = {-6.50, -2.75, 2.75, 6.50};
    public static final double TAG_DOWN = 7.1874, TAG_OUT = 5.622;

    /** Section 9.9 / SDK library: the first tag ID of each CELL's cluster. */
    public static int firstTagId(Alliance alliance, End end) {
        if (alliance == Alliance.RED) return end == End.SCORING ? 30 : 34;
        return end == End.AUDIENCE ? 38 : 42;
    }

    /** Setup Guide 11.1: red's audience CELL and blue's scoring CELL start facing up. */
    public static End upAtStart(Alliance alliance) {
        return alliance == Alliance.RED ? End.AUDIENCE : End.SCORING;
    }

    /** The HIVE's tilt about the field's x axis, positive with the scoring end raised. */
    public static double tiltRadians(End upEnd) {
        return Math.toRadians(upEnd == End.SCORING ? TILT_DEG : -TILT_DEG);
    }

    /**
     * A point given in the HIVE's frame (across = field x, along = towards
     * the scoring end, up = off the HIVE's floor), with the HIVE tilted so
     * {@code upEnd} is raised.
     */
    public static V3 hivePoint(Alliance alliance, End upEnd, double across, double along, double up) {
        double t = tiltRadians(upEnd);
        return new V3(hiveX(alliance) + across,
                CENTER + along * Math.cos(t) - up * Math.sin(t),
                PIVOT_HEIGHT + along * Math.sin(t) + up * Math.cos(t));
    }

    /** A direction given in the HIVE's frame, tilted with it. */
    public static V3 hiveDirection(End upEnd, double across, double along, double up) {
        double t = tiltRadians(upEnd);
        return new V3(across, along * Math.cos(t) - up * Math.sin(t), along * Math.sin(t) + up * Math.cos(t));
    }

    /** Where a CELL's opening is, the point a LAUNCH aims at. */
    public static V3 opening(Alliance alliance, End cell, End upEnd) {
        return hivePoint(alliance, upEnd, 0, cell.sign * OPENING_ALONG, OPENING_UP);
    }

    /**
     * A CELL's AprilTag Cluster frame, as the SDK's pose solver reports it:
     * x along the sticker (to the right, facing it), y down the sticker, z into
     * it. The sticker is on the CELL's floor facing the TILES, with its bottom
     * edge towards the center of the FIELD (Section 9.9).
     */
    public static V3[] clusterAxes(End cell, End upEnd) {
        V3 z = hiveDirection(upEnd, 0, 0, 1);
        V3 y = hiveDirection(upEnd, 0, -cell.sign, 0);
        return new V3[] {y.cross(z), y, z};
    }

    /** The center of member tag {@code index} (0..3, IDs in order) of a CELL's cluster. */
    public static V3 tagCenter(Alliance alliance, End cell, End upEnd, int index) {
        V3[] axes = clusterAxes(cell, upEnd);
        return opening(alliance, cell, upEnd)
                .plus(axes[0].times(TAG_ACROSS[index]))
                .plus(axes[1].times(TAG_DOWN))
                .plus(axes[2].times(-TAG_OUT));
    }

    /**
     * A CELL's outline seen from above, as (minY, maxY) of its box at this tilt;
     * across it spans {@link #CELL_WIDTH} centered on the HIVE.
     */
    public static double[] cellSpanY(Alliance alliance, End cell, End upEnd) {
        double lo = Double.MAX_VALUE, hi = -Double.MAX_VALUE;
        for (double along : new double[] {CELL_INNER, CELL_OUTER}) {
            for (double up : new double[] {0, CELL_HEIGHT}) {
                double y = hivePoint(alliance, upEnd, 0, cell.sign * along, up).y;
                lo = Math.min(lo, y);
                hi = Math.max(hi, y);
            }
        }
        return new double[] {lo, hi};
    }

    /**
     * Setup Guide 12: a HIVE tips when what is in its upward CELL weighs as
     * much as 3 NECTAR + 3 POLLEN or 8 POLLEN, but not 3 NECTAR + 2 POLLEN or
     * 7 POLLEN (calibration table 12.3). A POLLEN weighing 1 and a NECTAR 1.5,
     * tipping at 7.5, meets every row; the guide also says 3 + 3 is the lighter
     * of the two, which this keeps.
     */
    public static final double POLLEN_WEIGHT = 1.0, NECTAR_WEIGHT = 1.5, TIP_WEIGHT = 7.5;

    // --------------------------------------------------------------- the frame

    /**
     * Section 9.6.1: the frame stands on two triangles 49.46 in. apart (across
     * x), each with a 38.95 in. base along y. The HIVES are 30.6 in. off the
     * TILES at their lowest (Figure 9-10), so a ROBOT inside the 29 in. sizing
     * volume drives under them; only the two base bars are in its way.
     */
    public static final double FRAME_WIDTH = 49.46, FRAME_DEPTH = 38.95, FOOT_BAR = 1.5;
    public static final List<Rectangle2D.Double> FRAME_BARS = Collections.unmodifiableList(Arrays.asList(
            new Rectangle2D.Double(CENTER - FRAME_WIDTH / 2 - FOOT_BAR / 2, CENTER - FRAME_DEPTH / 2,
                    FOOT_BAR, FRAME_DEPTH),
            new Rectangle2D.Double(CENTER + FRAME_WIDTH / 2 - FOOT_BAR / 2, CENTER - FRAME_DEPTH / 2,
                    FOOT_BAR, FRAME_DEPTH)));

    // ------------------------------------------------------- around the walls

    /** The 4 FLOWERS, on the perimeter at TILE seams (Figure 9-2), one per wall. */
    public static final List<Point2D.Double> FLOWERS = Collections.unmodifiableList(Arrays.asList(
            new Point2D.Double(48, FIELD_INCHES),
            new Point2D.Double(FIELD_INCHES, 96),
            new Point2D.Double(96, 0),
            new Point2D.Double(0, 48)));

    /**
     * How far a FLOWER's base stands out from the wall, as a solid half-disc.
     * Section 9.7 gives the top opening (4 in.) and the POLLEN hole in its
     * lower ring (2.79 in.); the ring itself is a guess at 6 in. across.
     */
    public static final double FLOWER_RADIUS = 3.0;
    /** Section 10.3.1: 4 POLLEN in each FLOWER at the start. */
    public static final int FLOWER_POLLEN = 4;

    /** Setup Guide 8.3: 23 x 11 in. against the wall, red on TILE A5, blue on F2. */
    public static final Rectangle2D.Double RED_LOADING_ZONE = new Rectangle2D.Double(0, 96.5, 11, 23);
    public static final Rectangle2D.Double BLUE_LOADING_ZONE =
            new Rectangle2D.Double(FIELD_INCHES - 11, 24.5, 11, 23);

    /** Section 9.3 / Setup Guide 8.4: 23 x 2 in. along the wall, red in A1, blue in F6. */
    public static final Rectangle2D.Double RED_GARDEN = new Rectangle2D.Double(0, 0, 23, 2);
    public static final Rectangle2D.Double BLUE_GARDEN =
            new Rectangle2D.Double(FIELD_INCHES - 23, FIELD_INCHES - 2, 23, 2);

    // --------------------------------------------------------- SCORING ELEMENTS

    public enum BallType {
        POLLEN(2.8 / 2, POLLEN_WEIGHT), RED_NECTAR(3.6 / 2, NECTAR_WEIGHT), BLUE_NECTAR(3.6 / 2, NECTAR_WEIGHT);

        /** Section 9.8: POLLEN are 2.8 in., NECTAR 3.6 in. across. */
        public final double radius;
        public final double weight;

        BallType(double radius, double weight) {
            this.radius = radius;
            this.weight = weight;
        }

        public static BallType nectar(Alliance alliance) {
            return alliance == Alliance.RED ? RED_NECTAR : BLUE_NECTAR;
        }
    }

    /**
     * Section 10.3.1: the 4 POLLEN in each GARDEN, in a line from the corner
     * nearest that ALLIANCE's area, against the audience (red) or far (blue)
     * wall.
     */
    public static List<Point2D.Double> gardenPollen(Alliance alliance) {
        double r = BallType.POLLEN.radius;
        Point2D.Double[] at = new Point2D.Double[4];
        for (int i = 0; i < 4; i++) {
            double x = r + 2 * r * i;
            at[i] = alliance == Alliance.RED
                    ? new Point2D.Double(x, r)
                    : new Point2D.Double(FIELD_INCHES - x, FIELD_INCHES - r);
        }
        return Arrays.asList(at);
    }

    /** Section 10.3.1: 3 NECTAR of its color in each upward CELL; 4 POLLEN pre-loaded per ROBOT. */
    public static final int CELL_NECTAR_AT_START = 3, PRELOAD_POLLEN = 4;

    /** Rule G407: a ROBOT may not CONTROL more than 4 SCORING ELEMENTS. */
    public static final int ROBOT_CAPACITY = 4;

    // ---------------------------------------------------------- starting spots

    /** Where and which way a robot starts, for one ALLIANCE. */
    public static final class StartPose {
        public final String label;
        public final Alliance alliance;
        public final double x, y;
        /** Degrees clockwise from facing the far wall, as {@link RobotSim} measures it. */
        public final double headingDeg;

        public StartPose(String label, Alliance alliance, double x, double y, double headingDeg) {
            this.label = label;
            this.alliance = alliance;
            this.x = x;
            this.y = y;
            this.headingDeg = headingDeg;
        }
    }

    /**
     * Ready-made legal starting positions ({@link #isLegalStart}). The first is
     * the default: lined up with its own HIVE and facing the CELL that starts
     * up - red from the audience wall, blue from the far wall - so a pre-load
     * LAUNCHED from the start goes straight at the opening.
     */
    public enum StartSpot {
        RED_FACING_HIVE("Red, facing its HIVE (C1)", Alliance.RED, hiveX(Alliance.RED), 0),
        BLUE_FACING_HIVE("Blue, facing its HIVE (D6)", Alliance.BLUE, hiveX(Alliance.BLUE), 180),
        RED_AUDIENCE("Red, audience wall (B1)", Alliance.RED, 36, 0),
        RED_FAR("Red, far wall (B6)", Alliance.RED, 34, 180),
        BLUE_AUDIENCE("Blue, audience wall (E1)", Alliance.BLUE, 110, 0),
        BLUE_FAR("Blue, far wall (E6)", Alliance.BLUE, 110, 180);

        public final String label;
        public final Alliance alliance;
        public final double x;
        public final double headingDeg;

        StartSpot(String label, Alliance alliance, double x, double headingDeg) {
            this.label = label;
            this.alliance = alliance;
            this.x = x;
            this.headingDeg = headingDeg;
        }

        /** Touching the wall it backs onto: the robot's half-width off it. */
        public double y() {
            double half = RobotSim.ROBOT_INCHES / 2;
            return headingDeg == 0 ? half : FIELD_INCHES - half;
        }

        public StartPose pose() {
            return new StartPose(label, alliance, x, y(), headingDeg);
        }

        public StartSpot next() {
            return values()[(ordinal() + 1) % values().length];
        }
    }

    public static final StartSpot DEFAULT_START = StartSpot.RED_FACING_HIVE;

    /** How close counts as touching, for floating-point positions. */
    private static final double TOUCH = 0.05;

    /**
     * G304: fully inside the FIELD on its own ALLIANCE's side (columns A-C red,
     * D-F blue), touching the perimeter wall, not contacting a FLOWER, not in a
     * LOADING ZONE. (Starting size, pre-loads and being motionless are the
     * robot's business, not the spot's.)
     */
    public static boolean isLegalStart(Alliance alliance, double x, double y, double headingDeg) {
        double[][] corners = robotCorners(x, y, headingDeg);
        double minX = Double.MAX_VALUE, maxX = -Double.MAX_VALUE, minY = Double.MAX_VALUE, maxY = -Double.MAX_VALUE;
        for (double[] c : corners) {
            minX = Math.min(minX, c[0]);
            maxX = Math.max(maxX, c[0]);
            minY = Math.min(minY, c[1]);
            maxY = Math.max(maxY, c[1]);
        }
        if (minX < -TOUCH || minY < -TOUCH || maxX > FIELD_INCHES + TOUCH || maxY > FIELD_INCHES + TOUCH) return false;
        boolean ownSide = alliance == Alliance.RED ? maxX <= CENTER + TOUCH : minX >= CENTER - TOUCH;
        if (!ownSide) return false;
        boolean touchingWall = minX <= TOUCH || minY <= TOUCH
                || maxX >= FIELD_INCHES - TOUCH || maxY >= FIELD_INCHES - TOUCH;
        if (!touchingWall) return false;
        double half = RobotSim.ROBOT_INCHES / 2;
        for (Rectangle2D.Double zone : Arrays.asList(RED_LOADING_ZONE, BLUE_LOADING_ZONE)) {
            if (FieldSim.obbOutOfRect(x, y, headingDeg, half, zone) != null) return false;
        }
        for (Point2D.Double f : FLOWERS) {
            if (FieldSim.distanceToRobot(x, y, headingDeg, f.x, f.y) <= FLOWER_RADIUS) return false;
        }
        return true;
    }

    /**
     * The legal start nearest (x, y), for placing the robot with a click: on
     * the side of the FIELD the click is on, backed against a wall of that
     * side and facing away from it, slid along the wall clear of LOADING
     * ZONES and FLOWERS.
     */
    public static StartPose nearestLegalStart(double x, double y) {
        Alliance alliance = x < CENTER ? Alliance.RED : Alliance.BLUE;
        double half = RobotSim.ROBOT_INCHES / 2;
        double lo = alliance == Alliance.RED ? half : CENTER + half;
        double hi = alliance == Alliance.RED ? CENTER - half : FIELD_INCHES - half;
        double sideWallX = alliance == Alliance.RED ? half : FIELD_INCHES - half;
        double sideHeading = alliance == Alliance.RED ? 90 : -90;

        StartPose best = null;
        double bestDistance = Double.MAX_VALUE;
        for (double t = 0; t <= FIELD_INCHES; t += 0.25) {
            // Along the audience wall, the far wall, and this ALLIANCE's own wall.
            double[][] candidates = {
                    {t, half, 0},
                    {t, FIELD_INCHES - half, 180},
                    {sideWallX, t, sideHeading}};
            for (double[] c : candidates) {
                boolean alongEndWall = c[2] == 0 || c[2] == 180;
                if (alongEndWall && (c[0] < lo || c[0] > hi)) continue;
                if (!alongEndWall && (c[1] < half || c[1] > FIELD_INCHES - half)) continue;
                if (!isLegalStart(alliance, c[0], c[1], c[2])) continue;
                double d = Math.hypot(c[0] - x, c[1] - y);
                if (d < bestDistance) {
                    bestDistance = d;
                    best = new StartPose(alliance == Alliance.RED ? "Red" : "Blue", alliance, c[0], c[1], c[2]);
                }
            }
        }
        return new StartPose(best.label + ", placed (" + tileAt(best.x, best.y) + ")",
                alliance, best.x, best.y, best.headingDeg);
    }

    /** The robot's four corners at (x, y) turned {@code headingDeg}. */
    static double[][] robotCorners(double x, double y, double headingDeg) {
        double h = Math.toRadians(headingDeg);
        double half = RobotSim.ROBOT_INCHES / 2;
        double rx = Math.cos(h), ry = -Math.sin(h), fx = Math.sin(h), fy = Math.cos(h);
        double[][] out = new double[4][];
        int i = 0;
        for (int a : new int[] {-1, 1}) {
            for (int b : new int[] {-1, 1}) {
                out[i++] = new double[] {x + (rx * a + fx * b) * half, y + (ry * a + fy * b) * half};
            }
        }
        return out;
    }

    /** "A1" .. "F6" for the TILE (x, y) is on (Figure 9-5). */
    public static String tileAt(double x, double y) {
        int column = (int) Math.max(0, Math.min(5, Math.floor(x / TILE_INCHES)));
        int row = (int) Math.max(0, Math.min(5, Math.floor(y / TILE_INCHES)));
        return "" + (char) ('A' + column) + (row + 1);
    }

    // -------------------------------------------------------------- vectors

    /** A point or direction in field inches. */
    public static final class V3 {
        public final double x, y, z;

        public V3(double x, double y, double z) {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        public V3 plus(V3 o) { return new V3(x + o.x, y + o.y, z + o.z); }
        public V3 minus(V3 o) { return new V3(x - o.x, y - o.y, z - o.z); }
        public V3 times(double k) { return new V3(x * k, y * k, z * k); }
        public double dot(V3 o) { return x * o.x + y * o.y + z * o.z; }
        public double length() { return Math.sqrt(dot(this)); }

        public V3 cross(V3 o) {
            return new V3(y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x);
        }

        @Override public String toString() {
            return String.format(java.util.Locale.ROOT, "(%.2f, %.2f, %.2f)", x, y, z);
        }
    }
}

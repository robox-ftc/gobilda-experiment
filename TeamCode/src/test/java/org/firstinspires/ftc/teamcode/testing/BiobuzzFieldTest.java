package org.firstinspires.ftc.teamcode.testing;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.testing.BiobuzzField.Alliance;
import org.firstinspires.ftc.teamcode.testing.BiobuzzField.End;
import org.firstinspires.ftc.teamcode.testing.BiobuzzField.StartPose;
import org.firstinspires.ftc.teamcode.testing.BiobuzzField.StartSpot;
import org.firstinspires.ftc.teamcode.testing.BiobuzzField.V3;
import org.junit.Test;

import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;

/** The BIOBUZZ geometry agrees with the numbers the manual prints. */
public class BiobuzzFieldTest {

    private static final double F = BiobuzzField.FIELD_INCHES;

    @Test
    public void tilesAreLetteredFromTheRedWallAndNumberedFromTheAudience() {
        assertEquals("A1", BiobuzzField.tileAt(1, 1));
        assertEquals("F6", BiobuzzField.tileAt(F - 1, F - 1));
        assertEquals("A5", BiobuzzField.tileAt(5, 100));
        assertEquals("F2", BiobuzzField.tileAt(F - 5, 30));
    }

    @Test
    public void zonesAndGardensAreOnTheTilesTheSetupGuideNames() {
        assertEquals("A5", centerTile(BiobuzzField.RED_LOADING_ZONE));
        assertEquals("F2", centerTile(BiobuzzField.BLUE_LOADING_ZONE));
        assertEquals("A1", centerTile(BiobuzzField.RED_GARDEN));
        assertEquals("F6", centerTile(BiobuzzField.BLUE_GARDEN));
    }

    private static String centerTile(Rectangle2D.Double r) {
        return BiobuzzField.tileAt(r.getCenterX(), r.getCenterY());
    }

    @Test
    public void flowersAndLoadingZonesAreSymmetricUnderAHalfTurn() {
        for (Point2D.Double f : BiobuzzField.FLOWERS) {
            boolean mirrored = false;
            for (Point2D.Double o : BiobuzzField.FLOWERS) {
                if (Math.abs(o.x - (F - f.x)) < 1e-9 && Math.abs(o.y - (F - f.y)) < 1e-9) mirrored = true;
            }
            assertTrue("flower " + f + " has a partner across the center", mirrored);
        }
        assertEquals(F - BiobuzzField.RED_LOADING_ZONE.getCenterX(), BiobuzzField.BLUE_LOADING_ZONE.getCenterX(), 1e-9);
        assertEquals(F - BiobuzzField.RED_LOADING_ZONE.getCenterY(), BiobuzzField.BLUE_LOADING_ZONE.getCenterY(), 1e-9);
    }

    @Test
    public void theFrameBarsAre49_46ApartAnd38_95Long() {
        Rectangle2D.Double left = BiobuzzField.FRAME_BARS.get(0), right = BiobuzzField.FRAME_BARS.get(1);
        assertEquals(49.46, right.getCenterX() - left.getCenterX(), 1e-9);
        assertEquals(38.95, left.height, 1e-9);
        assertEquals(F / 2, left.getCenterY(), 1e-9);
        assertEquals(F / 2, (left.getCenterX() + right.getCenterX()) / 2, 1e-9);
    }

    @Test
    public void theHivesAre25_5ApartRedOnTheRedSide() {
        assertEquals(25.5, BiobuzzField.hiveX(Alliance.BLUE) - BiobuzzField.hiveX(Alliance.RED), 1e-9);
        assertTrue(BiobuzzField.hiveX(Alliance.RED) < F / 2);
    }

    @Test
    public void anUpwardCellsOpeningIsWhereFigure9_10PutsIt() {
        for (Alliance a : Alliance.values()) {
            End up = BiobuzzField.upAtStart(a);
            V3 o = BiobuzzField.opening(a, up, up);
            // Figure 9-10: the opening runs 53.5 to 65.6 in. above the TILES.
            assertEquals(a + " opening height", (53.5 + 65.6) / 2, o.z, 0.1);
            V3 down = BiobuzzField.opening(a, up.other(), up);
            assertTrue("the other CELL is lower", down.z < o.z - 15);
        }
    }

    @Test
    public void theCellsSitWhereFigure9_17DrawsThemAtTheStart() {
        // Measured off Figure 9-17 to about +/- 2 in.
        assertEquals(88.5, spanCenter(Alliance.RED, End.SCORING), 3);
        assertEquals(61.5, spanCenter(Alliance.RED, End.AUDIENCE), 3);
        assertEquals(83.0, spanCenter(Alliance.BLUE, End.SCORING), 3);
        assertEquals(56.0, spanCenter(Alliance.BLUE, End.AUDIENCE), 3);
    }

    private static double spanCenter(Alliance a, End cell) {
        double[] span = BiobuzzField.cellSpanY(a, cell, BiobuzzField.upAtStart(a));
        return (span[0] + span[1]) / 2;
    }

    @Test
    public void tagsRunLeftToRightAsFigure9_17ShowsThem() {
        assertTagOrderLeftToRight(Alliance.RED, End.SCORING, 33, 32, 31, 30);
        assertTagOrderLeftToRight(Alliance.RED, End.AUDIENCE, 34, 35, 36, 37);
        assertTagOrderLeftToRight(Alliance.BLUE, End.AUDIENCE, 38, 39, 40, 41);
        assertTagOrderLeftToRight(Alliance.BLUE, End.SCORING, 45, 44, 43, 42);
    }

    private static void assertTagOrderLeftToRight(Alliance a, End cell, int... ids) {
        int first = BiobuzzField.firstTagId(a, cell);
        double lastX = -Double.MAX_VALUE;
        for (int id : ids) {
            double x = BiobuzzField.tagCenter(a, cell, BiobuzzField.upAtStart(a), id - first).x;
            assertTrue(a + " " + cell + ": tag " + id + " after the one before it", x > lastX);
            lastX = x;
        }
    }

    @Test
    public void tagsFaceTheTilesFromTheCellsUnderside() {
        for (Alliance a : Alliance.values()) {
            for (End cell : End.values()) {
                for (End up : End.values()) {
                    V3 into = BiobuzzField.clusterAxes(cell, up)[2];
                    assertTrue("seen from below", into.z > 0.8);
                    V3 tag = BiobuzzField.tagCenter(a, cell, up, 0);
                    assertTrue("above the robot", tag.z > 30);
                }
            }
        }
    }

    @Test
    public void tippingMatchesTheCalibrationTable() {
        double p = BiobuzzField.POLLEN_WEIGHT, n = BiobuzzField.NECTAR_WEIGHT, tip = BiobuzzField.TIP_WEIGHT;
        assertTrue("3 NECTAR + 3 POLLEN tips", 3 * n + 3 * p >= tip);
        assertTrue("8 POLLEN tips", 8 * p >= tip);
        assertFalse("3 NECTAR + 2 POLLEN, gently placed, may not", 3 * n + 2 * p >= tip);
        assertFalse("7 POLLEN, gently placed, may not", 7 * p >= tip);
        assertTrue("3 + 3 is the lighter of the two", 3 * n + 3 * p < 8 * p);
    }

    @Test
    public void everyReadyMadeStartIsLegal() {
        for (StartSpot spot : StartSpot.values()) {
            assertTrue(spot.label, BiobuzzField.isLegalStart(spot.alliance, spot.x, spot.y(), spot.headingDeg));
        }
    }

    @Test
    public void theDefaultStartFacesItsOwnUpwardCellHeadOn() {
        for (StartSpot spot : new StartSpot[] {StartSpot.RED_FACING_HIVE, StartSpot.BLUE_FACING_HIVE}) {
            End up = BiobuzzField.upAtStart(spot.alliance);
            V3 o = BiobuzzField.opening(spot.alliance, up, up);
            RobotSim sim = new RobotSim();
            sim.setStartingPose(spot.x, spot.y(), spot.headingDeg);
            assertEquals(spot.label + " points at it", 0, sim.bearingTo(o.x, o.y), 1e-6);
            double gap = Math.abs(o.y - spot.y()) - RobotSim.ROBOT_INCHES / 2;
            assertTrue(spot.label + " within launching range, was " + gap,
                    gap >= FieldSim.MIN_SCORING_RANGE && gap <= FieldSim.MAX_SCORING_RANGE);
        }
        assertEquals(StartSpot.RED_FACING_HIVE, BiobuzzField.DEFAULT_START);
    }

    @Test
    public void g304RejectsWhatItShould() {
        double half = RobotSim.ROBOT_INCHES / 2;
        assertFalse("off the wall", BiobuzzField.isLegalStart(Alliance.RED, 40, 30, 0));
        assertFalse("on the other side", BiobuzzField.isLegalStart(Alliance.BLUE, 40, half, 0));
        assertFalse("straddling the center line", BiobuzzField.isLegalStart(Alliance.RED, F / 2, half, 0));
        assertFalse("in the LOADING ZONE", BiobuzzField.isLegalStart(Alliance.RED, half, 108, 90));
        assertFalse("against a FLOWER", BiobuzzField.isLegalStart(Alliance.RED, half, 48, 90));
        assertFalse("turned so a corner is through the wall", BiobuzzField.isLegalStart(Alliance.RED, 40, half, 30));
        assertTrue(BiobuzzField.isLegalStart(Alliance.RED, half, 30, 90));
    }

    @Test
    public void aClickAnywhereLandsOnALegalStartOnThatSide() {
        for (double x = 2; x < F; x += 7) {
            for (double y = 2; y < F; y += 7) {
                StartPose p = BiobuzzField.nearestLegalStart(x, y);
                assertEquals(x < F / 2 ? Alliance.RED : Alliance.BLUE, p.alliance);
                assertTrue("click at " + x + "," + y + " gave " + p.label,
                        BiobuzzField.isLegalStart(p.alliance, p.x, p.y, p.headingDeg));
            }
        }
    }

    @Test
    public void aClickInTheLoadingZoneOrOnAFlowerIsMovedClear() {
        StartPose zone = BiobuzzField.nearestLegalStart(5, 108);
        assertTrue("slid along the wall, out of the zone",
                zone.y + RobotSim.ROBOT_INCHES / 2 <= BiobuzzField.RED_LOADING_ZONE.y + 1e-6
                        || zone.y - RobotSim.ROBOT_INCHES / 2 >= BiobuzzField.RED_LOADING_ZONE.y
                        + BiobuzzField.RED_LOADING_ZONE.height - 1e-6);
        StartPose flower = BiobuzzField.nearestLegalStart(3, 48);
        assertTrue(Math.abs(flower.y - 48) > RobotSim.ROBOT_INCHES / 2 + BiobuzzField.FLOWER_RADIUS - 1e-6);
    }

    @Test
    public void aClickNearAWallBacksOntoItFacingIn() {
        StartPose p = BiobuzzField.nearestLegalStart(30, 3);
        assertEquals(RobotSim.ROBOT_INCHES / 2, p.y, 1e-9);
        assertEquals(0, p.headingDeg, 1e-9);
        StartPose side = BiobuzzField.nearestLegalStart(140, 70);
        assertEquals(F - RobotSim.ROBOT_INCHES / 2, side.x, 1e-9);
        assertEquals(-90, side.headingDeg, 1e-9);
    }
}

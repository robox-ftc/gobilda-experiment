package org.firstinspires.ftc.teamcode.testing;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.testing.BiobuzzField.Alliance;
import org.firstinspires.ftc.teamcode.testing.BiobuzzField.BallType;
import org.firstinspires.ftc.teamcode.testing.BiobuzzField.End;
import org.firstinspires.ftc.teamcode.testing.BiobuzzField.StartSpot;
import org.firstinspires.ftc.teamcode.testing.BiobuzzField.V3;
import org.junit.Test;

import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;

/** Balls, the robot's intake and launches, and HIVE tips. */
public class FieldSimTest {

    private static final double DT = 0.02;

    private final FieldSim field = new FieldSim();
    private final RobotSim sim = new RobotSim();

    private void run(double seconds, double intake) {
        for (double t = 0; t < seconds; t += DT) field.step(sim, intake, DT);
    }

    /** Puts the robot facing an upward CELL's opening, {@code distance} in. short of it. */
    private void aimAt(Alliance a, double distance) {
        End up = field.up(a);
        V3 o = BiobuzzField.opening(a, up, up);
        double half = RobotSim.ROBOT_INCHES / 2;
        sim.setStartingPose(o.x, o.y - up.sign * (distance + half), up == End.SCORING ? 0 : 180);
    }

    @Test
    public void theFieldIsStagedAsSection10_3_1Says() {
        assertEquals("8 POLLEN in the GARDENS", 8, field.balls().size());
        for (int i = 0; i < 4; i++) assertEquals(4, field.flowerPollen(i));
        for (Alliance a : Alliance.values()) {
            End up = BiobuzzField.upAtStart(a);
            assertEquals(up, field.up(a));
            assertEquals(3, field.cell(a, up).size());
            for (BallType t : field.cell(a, up)) assertEquals(BallType.nectar(a), t);
            assertTrue(field.cell(a, up.other()).isEmpty());
        }
        assertEquals("4 pre-loaded POLLEN", 4, field.carried());
    }

    @Test
    public void theRobotStartsOnItsSpot() {
        field.reset(StartSpot.BLUE_FAR);
        field.placeRobot(sim);
        assertEquals(StartSpot.BLUE_FAR.x, sim.xInches, 1e-9);
        assertEquals(StartSpot.BLUE_FAR.y(), sim.yInches, 1e-9);
        assertEquals(180, Math.abs(sim.headingDeg), 1e-9);
    }

    @Test
    public void threeNectarAndThreePollenTipTheHiveAndItSpills() {
        aimAt(Alliance.RED, 40);
        for (int i = 0; i < 2; i++) {
            assertEquals("RED AUDIENCE CELL", field.launch(sim, 0.8).cell);
        }
        assertEquals("2 POLLEN on 3 NECTAR: not yet", End.AUDIENCE, field.up(Alliance.RED));
        int before = field.balls().size();

        field.launch(sim, 0.8);

        assertEquals(End.SCORING, field.up(Alliance.RED));
        assertEquals(1, field.tips(Alliance.RED));
        assertTrue(field.cell(Alliance.RED, End.AUDIENCE).isEmpty());
        assertEquals("all 6 fall out", before + 6, field.balls().size());
    }

    @Test
    public void eightPollenTipAnEmptyCell() {
        // Blue's scoring CELL starts up with 3 NECTAR; tip it once to get an empty upward CELL.
        aimAt(Alliance.BLUE, 30);
        for (int i = 0; i < 3; i++) field.launch(sim, 0.8);
        assertEquals(End.AUDIENCE, field.up(Alliance.BLUE));

        aimAt(Alliance.BLUE, 30);
        for (int i = 0; i < 7; i++) field.launch(sim, 0.8);
        assertEquals("7 POLLEN do not", End.AUDIENCE, field.up(Alliance.BLUE));
        field.launch(sim, 0.8);
        assertEquals("the 8th does", End.SCORING, field.up(Alliance.BLUE));
    }

    @Test
    public void aShotThatIsNotAimedLandsOnTheFloorAhead() {
        aimAt(Alliance.RED, 40);
        sim.headingDeg += 60;
        int before = field.balls().size();

        FieldSim.Shot shot = field.launch(sim, 0.4);

        assertNull(shot.cell);
        assertEquals(before + 1, field.balls().size());
        double h = Math.toRadians(sim.headingDeg);
        double ahead = (shot.landX - sim.xInches) * Math.sin(h) + (shot.landY - sim.yInches) * Math.cos(h);
        assertTrue("landed in front, was " + ahead, ahead > 20);
    }

    @Test
    public void slowWheelsDoNotReachTheCell() {
        aimAt(Alliance.RED, 40);
        assertNull(field.launch(sim, 0.2).cell);
    }

    @Test
    public void theFrontPushesABallAndTheBackTakesItIn() {
        field.hold();
        sim.setStartingPose(36, 30, 0);
        FieldSim.Ball ahead = field.addBall(BallType.POLLEN, 36, 30 + RobotSim.ROBOT_INCHES / 2 + 1);
        for (int i = 0; i < 25; i++) {
            sim.step(0.5, -0.5, 0.5, -0.5, DT); // forward
            field.step(sim, 1.0, DT);
        }
        assertTrue("pushed, not swallowed", field.balls().contains(ahead));
        assertTrue("ahead of the robot", ahead.y > sim.yInches + RobotSim.ROBOT_INCHES / 2);
        assertEquals(0, field.carried());

        FieldSim.Ball behind = field.addBall(BallType.POLLEN, sim.xInches, sim.yInches - RobotSim.ROBOT_INCHES / 2 - 1);
        for (int i = 0; i < 25; i++) {
            sim.step(-0.5, 0.5, -0.5, 0.5, DT); // backward, into it
            field.step(sim, 1.0, DT);
        }
        assertFalse(field.balls().contains(behind));
        assertEquals(1, field.carried());
    }

    @Test
    public void theIntakeTakesNothingWhenOffOrFull() {
        field.hold(BallType.POLLEN, BallType.POLLEN, BallType.POLLEN, BallType.POLLEN);
        sim.setStartingPose(36, 30, 0);
        FieldSim.Ball behind = field.addBall(BallType.POLLEN, 36, 30 - RobotSim.ROBOT_INCHES / 2 - 0.5);
        run(0.2, 1.0);
        assertTrue("G407: no more than 4", field.balls().contains(behind));

        field.hold();
        run(0.2, 0.0);
        assertTrue("intake off", field.balls().contains(behind));
    }

    @Test
    public void theFrameBaseAndTheFlowersStopTheRobot() {
        Rectangle2D.Double bar = BiobuzzField.FRAME_BARS.get(0);
        sim.setStartingPose(bar.x - 4, bar.getCenterY(), 0);
        run(DT, 0);
        assertTrue("pushed clear of the bar", sim.xInches + RobotSim.ROBOT_INCHES / 2 <= bar.x + 1e-6);

        Point2D.Double flower = BiobuzzField.FLOWERS.get(2); // on the audience wall
        sim.setStartingPose(flower.x, RobotSim.ROBOT_INCHES / 2, 0);
        run(DT, 0);
        assertTrue("pushed clear of the FLOWER",
                Math.abs(sim.xInches - flower.x) >= RobotSim.ROBOT_INCHES / 2 + BiobuzzField.FLOWER_RADIUS - 1e-6
                        || sim.yInches >= RobotSim.ROBOT_INCHES / 2 + BiobuzzField.FLOWER_RADIUS - 1e-6);
    }

    @Test
    public void pollenComesOutOfAFlowersBottom() {
        field.hold();
        Point2D.Double flower = BiobuzzField.FLOWERS.get(2); // (96, 0), audience wall
        // Back up to it: facing the far wall, the intake at the back is at the wall.
        sim.setStartingPose(flower.x, BiobuzzField.FLOWER_RADIUS + RobotSim.ROBOT_INCHES / 2 + 0.5, 0);
        run(1.0, 1.0);
        assertTrue(field.carried() >= 2);
        assertEquals(4 - field.carried(), field.flowerPollen(2));
    }

    @Test
    public void reversingTheIntakeDropsTheBallBehind() {
        field.hold(BallType.POLLEN);
        sim.setStartingPose(72, 20, 0);
        assertTrue(field.takeForPath());
        int before = field.balls().size();
        field.spit(sim);
        assertEquals(before + 1, field.balls().size());
        FieldSim.Ball dropped = field.balls().get(field.balls().size() - 1);
        assertTrue(dropped.y < sim.yInches - RobotSim.ROBOT_INCHES / 2);
        assertEquals(0, field.carried());
    }

    @Test
    public void leaveIsOffTheWallAndParkIsInItsOwnLoadingZone() {
        field.reset(StartSpot.RED_FACING_HIVE);
        field.placeRobot(sim);
        assertFalse("starts touching the wall", FieldSim.offWall(sim));
        assertFalse(field.parked(sim));

        sim.setStartingPose(40, 40, 0);
        assertTrue(FieldSim.offWall(sim));

        Rectangle2D.Double red = BiobuzzField.RED_LOADING_ZONE;
        sim.setStartingPose(red.x + red.width + RobotSim.ROBOT_INCHES / 2 - 2, red.getCenterY(), 90);
        assertTrue("partly in is enough", field.parked(sim));

        Rectangle2D.Double blue = BiobuzzField.BLUE_LOADING_ZONE;
        sim.setStartingPose(blue.x, blue.getCenterY(), -90);
        assertFalse("blue's zone does not count for red", field.parked(sim));
    }

    @Test(expected = IllegalArgumentException.class)
    public void anIllegalStartIsRefused() {
        field.setStart(new BiobuzzField.StartPose("middle", Alliance.RED, 40, 40, 0));
    }
}

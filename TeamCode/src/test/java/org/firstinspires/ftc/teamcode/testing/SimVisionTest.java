package org.firstinspires.ftc.teamcode.testing;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.testing.BiobuzzField.Alliance;
import org.firstinspires.ftc.teamcode.testing.BiobuzzField.End;
import org.firstinspires.ftc.teamcode.testing.BiobuzzField.V3;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.junit.Test;

import java.util.List;

/** The simulated webcam reports what the SDK would, from where the robot is. */
public class SimVisionTest {

    private final SimVision vision = new SimVision();
    private final RobotSim sim = new RobotSim();
    private final FieldSim field = new FieldSim();

    /** 14 in. up, 5 in. forward, tilted 30 degrees up from level (SDK pitch -60). */
    private void mountLookingUpAhead() {
        vision.setMount(new Position(DistanceUnit.INCH, 0, 5, 14, 0),
                new YawPitchRollAngles(AngleUnit.DEGREES, 0, -60, 0, 0));
    }

    /** The robot facing red's upward (audience) CELL from the audience side, offset across. */
    private void faceRedAudienceCell(double across) {
        V3 o = BiobuzzField.opening(Alliance.RED, End.AUDIENCE, End.AUDIENCE);
        sim.setStartingPose(o.x + across, o.y - 50, 0);
        vision.update(sim, field);
    }

    private AprilTagClusterDetection only(List<AprilTagDetection> seen, String name) {
        for (AprilTagDetection d : seen) {
            AprilTagClusterDetection c = (AprilTagClusterDetection) d;
            if (c.metadata.name.equals(name)) return c;
        }
        throw new AssertionError(name + " not seen; saw " + seen.size());
    }

    @Test
    public void itSeesNothingUntilItKnowsWhereTheRobotIs() {
        assertTrue(vision.detections().isEmpty());
    }

    @Test
    public void aCellAheadAndAboveIsSeenAsItsCluster() {
        mountLookingUpAhead();
        faceRedAudienceCell(0);

        AprilTagClusterDetection d = only(vision.detections(), "RED AUDIENCE");

        assertTrue(d.percentClusterFound > 0);
        V3 o = BiobuzzField.opening(Alliance.RED, End.AUDIENCE, End.AUDIENCE);
        double straightLine = o.minus(vision.cameraFrame()[3]).length();
        double reported = Math.sqrt(d.ftcPose.x * d.ftcPose.x + d.ftcPose.y * d.ftcPose.y + d.ftcPose.z * d.ftcPose.z);
        assertEquals("distance to the opening", straightLine, reported, 1e-3);
        assertEquals("dead ahead", 0, d.ftcPose.x, 1e-3);
        assertTrue("above the lens's level", d.ftcPose.z > 0 || d.ftcPose.elevation > 0);
        for (int id = 34; id <= 37; id++) {
            if (vision.visibleTags().contains(id)) return;
        }
        throw new AssertionError("no RED AUDIENCE tag marked visible");
    }

    @Test
    public void aCellToTheRightHasPositiveXAndNegativeBearing() {
        mountLookingUpAhead();
        faceRedAudienceCell(-8); // robot left of it, so the CELL is to its right

        AprilTagClusterDetection d = only(vision.detections(), "RED AUDIENCE");

        assertTrue("x is right of the lens, was " + d.ftcPose.x, d.ftcPose.x > 0);
        assertTrue("SDK bearing is positive to the left, was " + d.ftcPose.bearing, d.ftcPose.bearing < 0);
    }

    @Test
    public void facingAwayItSeesNothing() {
        mountLookingUpAhead();
        faceRedAudienceCell(0);
        sim.headingDeg = 180;
        vision.update(sim, field);

        assertTrue(vision.detections().isEmpty());
    }

    @Test
    public void tipsMoveTheTags() {
        mountLookingUpAhead();
        faceRedAudienceCell(0);
        only(vision.detections(), "RED AUDIENCE");

        // Tip red's HIVE: its audience CELL swings down and away from where it was.
        field.hold();
        V3 o = BiobuzzField.opening(Alliance.RED, End.AUDIENCE, End.AUDIENCE);
        RobotSim shooter = new RobotSim();
        shooter.setStartingPose(o.x, o.y + 40 + RobotSim.ROBOT_INCHES / 2, 180);
        for (int i = 0; i < 3; i++) field.launch(shooter, 0.8);
        assertEquals(End.SCORING, field.up(Alliance.RED));

        // Tipped down, its tags face the field's center, away from this robot.
        vision.update(sim, field);
        for (AprilTagDetection d : vision.detections()) {
            AprilTagClusterDetection c = (AprilTagClusterDetection) d;
            assertTrue("RED AUDIENCE no longer in view", !c.metadata.name.equals("RED AUDIENCE"));
        }
        for (int id = 34; id <= 37; id++) assertTrue(!vision.visibleTags().contains(id));
    }

    @Test
    public void anOpModesCameraPoseReachesTheVision() {
        DesktopRobot robot = new DesktopRobot();
        try (SimCamera camera = SimCamera.install(robot.vision)) {
            new AprilTagProcessor.Builder()
                    .setCameraPose(new Position(DistanceUnit.INCH, 1, 2, 3, 0),
                            new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0))
                    .build();
        }
        robot.vision.update(new RobotSim(), null);
        V3[] cam = robot.vision.cameraFrame();
        assertEquals("pitch -90 looks forward", 1.0, cam[2].y, 1e-6);
        assertEquals(RobotSim.FIELD_INCHES / 2 + 1, cam[3].x, 1e-6);
        assertEquals(RobotSim.FIELD_INCHES / 2 + 2, cam[3].y, 1e-6);
        assertEquals(3, cam[3].z, 1e-6);
    }
}

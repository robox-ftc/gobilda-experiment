package org.firstinspires.ftc.teamcode.testing;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.devices.Drivetrain;
import org.firstinspires.ftc.teamcode.devices.DrivetrainControls;
import org.junit.Test;

/**
 * Pins down which way the robot moves on the field view.
 *
 * <p>These go through the real Drivetrain and its mecanum formula, built from
 * the desktop hardware map, out to the recorded motor powers, and only then into
 * {@link RobotSim}. So they fail if someone changes a sign in
 * {@code computeDriveTrainPower} as well as if the view's own kinematics are
 * wrong - a field view that silently agreed with the intent would be worthless.
 */
public class RobotSimTest {

    private static final double EPS = 1e-6;

    /** Long enough for the acceleration ramp to be clear of zero. */
    private static final double DT = 0.05;

    private final DesktopRobot robot = new DesktopRobot();
    private final Drivetrain drivetrain =
            new Drivetrain(robot.hardwareMap, robot.telemetry.device, false);
    private final RobotSim sim = new RobotSim();

    private void drive(double strafe, double forward, double turn, int cycles) {
        for (int i = 0; i < cycles; i++) {
            drivetrain.run(new DrivetrainControls(strafe, forward, turn));
            sim.step(robot.frontLeft.power, robot.frontRight.power,
                    robot.rearLeft.power, robot.rearRight.power, DT);
        }
    }

    @Test
    public void theWheelsDecomposeBackIntoTheCommand() {
        // Forward is (+,-,+,-) because the right motors are reversed in the
        // robot configuration rather than in code.
        sim.step(1.0, -1.0, 1.0, -1.0, DT);
        assertEquals("forward", 1.0, sim.forwardCmd, EPS);
        assertEquals("strafe", 0.0, sim.strafeCmd, EPS);
        assertEquals("turn", 0.0, sim.turnCmd, EPS);
    }

    @Test
    public void strafeAndTurnAreToldApart() {
        sim.step(1.0, 1.0, -1.0, -1.0, DT);
        assertEquals("strafe", 1.0, sim.strafeCmd, EPS);
        assertEquals("no turn", 0.0, sim.turnCmd, EPS);

        sim.step(1.0, 1.0, 1.0, 1.0, DT);
        assertEquals("turn", 1.0, sim.turnCmd, EPS);
        assertEquals("no strafe", 0.0, sim.strafeCmd, EPS);
    }

    @Test
    public void forwardMovesUpTheField() {
        drive(0, 1, 0, 10);

        assertTrue("y should grow, was " + sim.yInches, sim.yInches > RobotSim.FIELD_INCHES / 2 + 1);
        assertEquals("no sideways drift", RobotSim.FIELD_INCHES / 2, sim.xInches, 0.01);
        assertEquals("no rotation", 0.0, sim.headingDeg, 0.01);
    }

    @Test
    public void strafingRightMovesRight() {
        drive(1, 0, 0, 10);

        assertTrue("x should grow, was " + sim.xInches, sim.xInches > RobotSim.FIELD_INCHES / 2 + 1);
        assertEquals("no forward drift", RobotSim.FIELD_INCHES / 2, sim.yInches, 0.01);
    }

    @Test
    public void positiveRotationTurnsClockwiseOnTheSpot() {
        drive(0, 0, 1, 10);

        assertTrue("heading should grow clockwise, was " + sim.headingDeg, sim.headingDeg > 1.0);
        assertEquals(RobotSim.FIELD_INCHES / 2, sim.xInches, 0.01);
        assertEquals(RobotSim.FIELD_INCHES / 2, sim.yInches, 0.01);
    }

    @Test
    public void forwardFollowsTheHeading() {
        sim.headingDeg = 90; // facing along +x
        drive(0, 1, 0, 10);

        assertTrue("x should grow", sim.xInches > RobotSim.FIELD_INCHES / 2 + 1);
        assertEquals("y should hold", RobotSim.FIELD_INCHES / 2, sim.yInches, 0.01);
    }

    @Test
    public void theRobotStopsAtTheWall() {
        drive(0, 1, 0, 400);

        assertEquals(RobotSim.FIELD_INCHES - RobotSim.ROBOT_INCHES / 2, sim.yInches, EPS);
        assertTrue("should say it is pinned", sim.againstWall);
    }

    @Test
    public void resetPutsItBackInTheMiddle() {
        drive(0, 1, 0, 20);
        sim.reset();

        assertEquals(RobotSim.FIELD_INCHES / 2, sim.xInches, EPS);
        assertEquals(RobotSim.FIELD_INCHES / 2, sim.yInches, EPS);
        assertEquals(0.0, sim.headingDeg, EPS);
    }

    @Test
    public void theGoalBearingFollowsTheChassis() {
        double before = sim.bearingToGoal();
        sim.headingDeg = 30;

        assertEquals(before - 30, sim.bearingToGoal(), EPS);
    }
}

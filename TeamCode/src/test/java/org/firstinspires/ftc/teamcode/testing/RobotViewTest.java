package org.firstinspires.ftc.teamcode.testing;

import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.BIOBUZZAuto;
import org.firstinspires.ftc.teamcode.devices.GamePadReadings;
import org.junit.After;
import org.junit.Test;

import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.util.Arrays;

/**
 * Paints the field view off screen, with the real auto running, so a repaint
 * that throws is caught here rather than showing up as a blank panel someone
 * blames on the robot code. No window and no controller needed.
 */
public class RobotViewTest {

    private static final int W = 1240;
    private static final int H = 660;
    private static final double DT = 0.05;

    private final DriverStation ds = new DriverStation();
    private DesktopRobot robot = new DesktopRobot();
    private OpModeRunner runner = new OpModeRunner(BIOBUZZAuto.class, robot, ds);
    private final RobotSim sim = new RobotSim();
    private final BallPath ball = new BallPath();
    private final RobotView view = new RobotView();

    @After
    public void close() {
        runner.close();
    }

    private BufferedImage paint(int width, int height) {
        BufferedImage image = new BufferedImage(width, height, BufferedImage.TYPE_INT_RGB);
        Graphics2D g = image.createGraphics();
        try {
            view.renderOffscreen(g, width, height);
        } finally {
            g.dispose();
        }
        return image;
    }

    private void cycle() {
        GamePadReadings readings = runner.cycle(DT);
        sim.step(robot.frontLeft.power, robot.frontRight.power,
                robot.rearLeft.power, robot.rearRight.power, DT);
        ball.step(robot.intakeFlow(), robot.rampFlow(), robot.flywheelSpeed(), DT);
        view.update(readings, robot, sim, ball, robot.telemetry.visible(), runner);
    }

    private static boolean drewSomething(BufferedImage image) {
        int first = image.getRGB(0, 0);
        for (int y = 0; y < image.getHeight(); y += 3) {
            for (int x = 0; x < image.getWidth(); x += 3) {
                if (image.getRGB(x, y) != first) return true;
            }
        }
        return false;
    }

    private static int signature(BufferedImage image) {
        return Arrays.hashCode(image.getRGB(0, 0, image.getWidth(), image.getHeight(),
                null, 0, image.getWidth()));
    }

    @Test
    public void aViewThatHasNeverBeenUpdatedStillPaints() {
        assertTrue(drewSomething(paint(W, H)));
    }

    @Test
    public void itPaintsInInit() {
        cycle();
        assertTrue(drewSomething(paint(W, H)));
    }

    @Test
    public void itPaintsTheRobotMovingAndTheFrameChanges() {
        cycle();
        BufferedImage before = paint(W, H);

        runner.start();
        for (int i = 0; i < 10; i++) cycle(); // Leave: forward for half a second
        BufferedImage after = paint(W, H);

        assertNotEquals("the robot should have visibly moved", signature(before), signature(after));
    }

    @Test
    public void itPaintsAStoppedAuto() {
        runner.start();
        for (int i = 0; i < 620; i++) cycle(); // past 30 s

        assertTrue(runner.phase() == OpModeRunner.Phase.STOPPED);
        assertTrue(drewSomething(paint(W, H)));
    }

    @Test
    public void itPaintsACrash() {
        runner.close();
        robot = new DesktopRobot("intake");
        runner = new OpModeRunner(BIOBUZZAuto.class, robot, ds);
        runner.start();
        for (int i = 0; i < 20; i++) cycle();

        assertNotNull("the drivetrain-only fallback crashes after its first move", runner.failure());
        assertTrue(drewSomething(paint(W, H)));
    }

    @Test
    public void itPaintsAgainstTheWallWithALongTrail() {
        robot.frontLeft.device.setPower(1);
        robot.frontRight.device.setPower(-1);
        robot.rearLeft.device.setPower(1);
        robot.rearRight.device.setPower(-1);
        for (int i = 0; i < 400; i++) {
            sim.step(1, -1, 1, -1, DT);
            view.update(new GamePadReadings(), robot, sim, ball, robot.telemetry.visible(), runner);
        }

        assertTrue(sim.againstWall);
        assertTrue(drewSomething(paint(W, H)));
    }

    @Test
    public void itPaintsTheNoControllerWarning() {
        view.setInputStatus("NO CONTROLLER FOUND - S still runs it with no choices made");
        cycle();
        assertTrue(drewSomething(paint(W, H)));
    }

    @Test
    public void itPaintsWithoutARunner() {
        view.update(new GamePadReadings(), robot, sim, ball, robot.telemetry.visible(), null);
        assertTrue(drewSomething(paint(W, H)));
    }

    @Test
    public void itSurvivesBeingResizedToNothing() {
        cycle();
        paint(400, 300);
        paint(200, 120);
        paint(60, 40);
        paint(1, 1);
    }
}

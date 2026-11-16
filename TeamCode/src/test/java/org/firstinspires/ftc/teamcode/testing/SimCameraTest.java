package org.firstinspires.ftc.teamcode.testing;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.NewAuto;
import org.firstinspires.ftc.teamcode.TeleOp;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.junit.Test;

/** OpModes that set up an AprilTag camera, which needs OpenCV, run on the PC. */
public class SimCameraTest {

    private static final double DT = 0.02;

    private OpModeRunner runThrough(Class<? extends OpMode> type) {
        OpModeRunner runner = new OpModeRunner(type, new DesktopRobot(), new DriverStation());
        for (int i = 0; i < 10; i++) runner.cycle(DT);
        runner.start();
        for (int i = 0; i < 100; i++) runner.cycle(DT);
        return runner;
    }

    @Test
    public void theCameraTeleOpGetsThroughInitAndRuns() {
        try (OpModeRunner runner = runThrough(TeleOp.class)) {
            assertNull(GamepadHarness.phaseLine(runner), runner.failure());
            assertEquals(OpModeRunner.Phase.RUNNING, runner.phase());
        }
    }

    @Test
    public void theCameraAutoGetsThroughInitAndRuns() {
        try (OpModeRunner runner = runThrough(NewAuto.class)) {
            assertNull(GamepadHarness.phaseLine(runner), runner.failure());
        }
    }

    @Test
    public void cameraSetupReportsNoError() {
        DesktopRobot robot = new DesktopRobot();
        try (OpModeRunner runner = new OpModeRunner(TeleOp.class, robot, new DriverStation())) {
            runner.cycle(DT);
            for (String line : robot.telemetry.visible()) {
                assertFalse(line, line.contains("Camera failed"));
            }
        }
    }

    @Test
    public void theStandInSeesNoTagsAndThePortalIsNull() {
        try (SimCamera camera = SimCamera.install()) {
            AprilTagProcessor processor = new AprilTagProcessor.Builder().setDrawAxes(true).build();
            VisionPortal portal = new VisionPortal.Builder().addProcessor(processor).build();
            assertTrue(processor.getDetections().isEmpty());
            assertNull(portal);
        }
    }
}

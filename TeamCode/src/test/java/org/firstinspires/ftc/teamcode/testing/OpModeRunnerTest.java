package org.firstinspires.ftc.teamcode.testing;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.junit.After;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

/** The runner itself: the SDK's call order, its clock, and what happens at the end. */
public class OpModeRunnerTest {

    /** Every call the runner made, in order. Static: the runner builds the instance. */
    static final List<String> calls = new ArrayList<>();
    static double timerAtLastLoop;
    static boolean sawAPress;

    /** Unannotated, so each subclass carries exactly one Driver Station annotation. */
    public abstract static class Recording extends OpMode {
        private final ElapsedTime timer = new ElapsedTime();
        private DcMotorEx motor;

        @Override public void init() {
            calls.add("init");
            motor = hardwareMap.get(DcMotorEx.class, "lf");
            telemetry.addLine("from init");
        }
        @Override public void init_loop() {
            calls.add("init_loop");
            sawAPress |= gamepad1.aWasPressed();
        }
        @Override public void start() {
            calls.add("start");
            timer.reset();
        }
        @Override public void loop() {
            calls.add("loop");
            motor.setPower(0.5);
            timerAtLastLoop = timer.seconds();
        }
        @Override public void stop() {
            calls.add("stop");
        }
    }

    @Autonomous(name = "Recording auto")
    public static class RecordingAuto extends Recording {}

    @TeleOp(name = "Recording teleop")
    public static class RecordingTeleop extends Recording {}

    public static class CrashingOpMode extends OpMode {
        @Override public void init() {}
        @Override public void loop() {
            throw new IllegalStateException("boom");
        }
    }

    private final DriverStation ds = new DriverStation();
    private final DesktopRobot robot = new DesktopRobot();
    private OpModeRunner runner;

    private OpModeRunner run(Class<? extends OpMode> type) {
        calls.clear();
        timerAtLastLoop = -1;
        sawAPress = false;
        runner = new OpModeRunner(type, robot, ds);
        return runner;
    }

    @After
    public void close() {
        if (runner != null) runner.close();
    }

    @Test
    public void theCallsComeInTheSdksOrder() {
        run(RecordingAuto.class);
        runner.cycle(0.02);
        runner.cycle(0.02);
        runner.start();
        runner.cycle(0.02);
        runner.stop();

        assertEquals(List.of("init", "init_loop", "init_loop", "start", "loop", "stop"), calls);
    }

    @Test
    public void initsTelemetryShowsOnlyOnceSomethingUpdatesIt() {
        // The SDK does not update telemetry after init(); it does after
        // init_loop(). So a line from init() appears with the first cycle.
        run(RecordingAuto.class);
        assertFalse(robot.telemetry.visible().contains("from init"));

        runner.cycle(0.02);
        assertTrue(robot.telemetry.visible().contains("from init"));
    }

    @Test
    public void theOpModesTimersRunOnRobotTime() {
        run(RecordingAuto.class);
        runner.start();
        for (int i = 0; i < 100; i++) runner.cycle(0.02);

        // The 100th loop ran when 99 steps of 20 ms had passed since start().
        assertEquals(1.98, timerAtLastLoop, 1e-9);
    }

    @Test
    public void gamepadEdgesReachTheOpMode() {
        run(RecordingAuto.class);
        ds.held1.a = true;
        runner.cycle(0.02);

        assertTrue(sawAPress);
    }

    @Test
    public void anAutonomousStopsAtThirtySecondsAndTheMotorsStop() {
        run(RecordingAuto.class);
        runner.start();
        for (int i = 0; i < 1600; i++) runner.cycle(0.02); // 32 s

        assertEquals(OpModeRunner.Phase.STOPPED, runner.phase());
        assertEquals("stop", calls.get(calls.size() - 1));
        assertEquals(0.0, robot.frontLeft.power, 1e-9);
    }

    @Test
    public void aTeleopKeepsGoing() {
        run(RecordingTeleop.class);
        runner.start();
        for (int i = 0; i < 1600; i++) runner.cycle(0.02);

        assertEquals(OpModeRunner.Phase.RUNNING, runner.phase());
        assertEquals(0.5, robot.frontLeft.power, 1e-9);
    }

    @Test
    public void aCrashStopsTheOpModeAndIsKept() {
        run(CrashingOpMode.class);
        runner.start();
        runner.cycle(0.02);
        Throwable first = runner.failure();
        runner.cycle(0.02);

        assertEquals(OpModeRunner.Phase.STOPPED, runner.phase());
        assertTrue(first instanceof IllegalStateException);
        assertSame("the first failure is the one kept", first, runner.failure());
    }

    @Test
    public void theNameIsTheOneTheDriverStationShows() {
        assertEquals("Recording auto", run(RecordingAuto.class).name());
        runner.close();
        assertEquals("CrashingOpMode", run(CrashingOpMode.class).name());
    }
}

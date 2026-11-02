package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.devices.Launcher;

import java.util.Arrays;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * The robot's hardware, on the desktop: a stand-in for every device TeamCode on
 * this branch looks up, registered under its configuration name in a
 * {@link DesktopHardwareMap}, plus the clock and the telemetry an OpMode is given.
 *
 * <p>Nothing here knows which OpMode will run. {@link OpModeRunner} hands
 * {@link #hardwareMap} to a real OpMode, which builds the real device classes
 * from it exactly as it does on the robot.
 */
public final class DesktopRobot {

    public final FakeHardware.Motor frontLeft = FakeHardware.motor("lf");
    public final FakeHardware.Motor frontRight = FakeHardware.motor("rf");
    public final FakeHardware.Motor rearLeft = FakeHardware.motor("lb");
    public final FakeHardware.Motor rearRight = FakeHardware.motor("rb");
    public final FakeHardware.Motor intakeWheel = FakeHardware.motor("intake");
    public final FakeHardware.Motor launcherLeft = FakeHardware.motor("leftLauncher");
    public final FakeHardware.Motor launcherRight = FakeHardware.motor("rightLauncher");
    public final FakeHardware.Motor rampWheel = FakeHardware.motor("ramp");

    /** Robot time. Every ElapsedTime an OpMode makes reads this while it runs. */
    public final SimClock clock = new SimClock();

    /** Driver Station telemetry, on robot time rather than the PC's. */
    public final FakeHardware.TelemetryRecorder telemetry = new FakeHardware.TelemetryRecorder(clock::nanos);

    /** What an OpMode is given as {@code hardwareMap}. */
    public final HardwareMap hardwareMap;

    /*
     * The launcher wheels are the one place where "what was commanded" and
     * "what the code can read back" differ enough to matter. Launcher on this
     * branch drives them open loop with setPower and telemeters
     * getVelocity() / RRP * 60 as RPM.
     *
     * ALL THREE CONSTANTS ARE GUESSES. The free speed comes from the comment
     * beside Launcher.RRP (a 5.1:1 goBILDA, 1150 rpm), and the time constants
     * from nothing at all - no one has measured this flywheel. Read the shape of
     * the spin-up off the screen, never the number.
     */
    public static final double LAUNCHER_FREE_RPM = 1150;
    public static final double LAUNCHER_TAU_SECONDS = 0.35;
    /** Launcher sets FLOAT, so a setPower(0) lets the wheels coast down. */
    public static final double LAUNCHER_COAST_TAU_SECONDS = 2.0;
    public static final FakeHardware.Plant LAUNCHER_PLANT = new FakeHardware.Plant(
            LAUNCHER_FREE_RPM / 60.0 * Launcher.RRP,
            LAUNCHER_TAU_SECONDS, LAUNCHER_COAST_TAU_SECONDS);

    /**
     * @param unplugged configuration names to leave out, as if they were
     *     missing from the robot configuration. Looking one up then throws the
     *     SDK's IllegalArgumentException, which is how a robot without that
     *     device behaves.
     */
    public DesktopRobot(String... unplugged) {
        launcherLeft.plant = LAUNCHER_PLANT;
        launcherRight.plant = LAUNCHER_PLANT;

        Set<String> missing = new HashSet<>(Arrays.asList(unplugged));
        Map<String, Object> devices = new LinkedHashMap<>();
        for (FakeHardware.Motor motor : motors()) {
            if (!missing.remove(motor.name)) devices.put(motor.name, motor.device);
        }
        if (!missing.isEmpty()) {
            throw new IllegalArgumentException("no such device to unplug: " + missing);
        }
        hardwareMap = DesktopHardwareMap.of(devices);
    }

    public List<FakeHardware.Motor> motors() {
        return Arrays.asList(frontLeft, frontRight, rearLeft, rearRight,
                intakeWheel, launcherLeft, launcherRight, rampWheel);
    }

    /**
     * Advances the modelled hardware, so the robot code has something to read
     * back next cycle. Nothing outside the launcher wheels is modelled.
     */
    public void step(double dtSec) {
        launcherLeft.step(dtSec);
        launcherRight.step(dtSec);
    }

    /*
     * Which way a ball is being moved, positive towards the flywheels. Read off
     * the recorded motors rather than anyone's intent, for the same reason
     * RobotSim reads the drive motors. On this branch Intake runs its motor
     * FORWARD and passes power straight through; Ramp negates what it is given.
     */

    /** The intake's pull on a ball, -1..1, positive towards the flywheels. */
    public double intakeFlow() {
        return intakeWheel.power;
    }

    /** The ramp's push on a ball, -1..1, positive towards the flywheels. */
    public double rampFlow() {
        return -rampWheel.power;
    }

    /** The left wheel's command, as a fraction of its free speed. */
    public double flywheelCommand() {
        return launcherLeft.velocityCommanded
                ? launcherLeft.velocity / LAUNCHER_PLANT.freeTicksPerSec
                : launcherLeft.power;
    }

    /** The left wheel's modelled speed, as a fraction of its free speed. A guess. */
    public double flywheelSpeed() {
        return launcherLeft.simTicksPerSec / LAUNCHER_PLANT.freeTicksPerSec;
    }

    /** RPM as Launcher.getLeftWheelRPM() would read it. Modelled, and a guess. */
    public double leftWheelRPM() {
        return launcherLeft.device.getVelocity() / Launcher.RRP * 60;
    }

    /** The RPM the current command is heading for, in the same units. */
    public double commandedWheelRPM() {
        return flywheelCommand() * LAUNCHER_FREE_RPM;
    }
}

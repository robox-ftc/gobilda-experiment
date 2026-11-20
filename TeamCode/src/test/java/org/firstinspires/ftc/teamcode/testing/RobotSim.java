package org.firstinspires.ftc.teamcode.testing;

/**
 * Works out somewhere to draw the robot from the powers the code commanded.
 *
 * <p><b>This is dead reckoning, not physics.</b> There is no mass, no wheel
 * slip, no floor, no ball and no opposing robot. Wheel powers are treated as
 * wheel speeds, run through mecanum kinematics, and integrated. It answers
 * "does this stick send the robot the way I meant, and do the four wheel signs
 * agree with each other" — which is a question about {@code
 * Drivetrain.computeDriveTrainPower}, not about the field. Do not read a
 * distance or a time off this screen and expect the real robot to match.
 *
 * <p>The input is the four <em>recorded</em> motor powers rather than the
 * {@link org.firstinspires.ftc.teamcode.devices.DrivetrainControls} that
 * produced them. Reading the controls would make this agree with the driver's
 * intent by construction; reading the motors means a sign error in the mecanum
 * formula, or a swapped motor, shows up as the robot going somewhere wrong.
 */
public final class RobotSim {

    /** FTC field, wall to wall. See {@link BiobuzzField}. */
    public static final double FIELD_INCHES = BiobuzzField.FIELD_INCHES;

    /** Rough footprint of this robot, for drawing only. */
    public static final double ROBOT_INCHES = 17.0;


    /*
     * The goBILDA 2026-27 StarterBot mecanum base: a Strafer Chassis Kit
     * (3209-0001-0007) with four 312 RPM 5203 Yellow Jacket motors (19.2:1)
     * on 104 mm GripForce mecanum wheels. goBILDA quotes its theoretical speed
     * as 5.57 ft/s - 312 RPM x pi x 104 mm = 66.8 in/s.
     *
     * Turning: a mecanum chassis spins at wheel speed / (half track + half
     * wheelbase). The wheelbase is 336 mm (shafts 48 mm from each end of the
     * 432 mm side channels, assembly guide Figure 5-A); the track is not
     * dimensioned and is taken as about 400 mm. That gives 264 deg/s.
     *
     * A loaded robot does not reach free speed; LOADED_FRACTION is an
     * assumption, not a measurement. Strafing is modelled at full speed, which
     * a real mecanum base usually falls short of. Measure the real robot -
     * one second at fixed power forward, sideways and turning, read off the
     * Pinpoint - and put the numbers here. The "Cal 3: Drive Speed" OpMode
     * does exactly that: run it on the robot and here, and copy its
     * full-power forward speed into MAX_SPEED_IPS (replacing the formula), its
     * sideways percentage into STRAFE_FRACTION, and its turning speed into
     * MAX_TURN_DPS.
     */
    public static final double FREE_SPEED_IPS = 312.0 / 60.0 * Math.PI * 104.0 / 25.4;
    public static final double WHEELBASE_INCHES = 336.0 / 25.4, TRACK_INCHES = 400.0 / 25.4;
    public static final double LOADED_FRACTION = 0.85;
    public static final double MAX_SPEED_IPS = FREE_SPEED_IPS * LOADED_FRACTION;
    /** Sideways speed as a fraction of forward speed. 1.0 is a guess; real mecanum bases are usually slower sideways. */
    public static final double STRAFE_FRACTION = 1.0;
    public static final double MAX_TURN_DPS =
            Math.toDegrees(MAX_SPEED_IPS / (TRACK_INCHES / 2 + WHEELBASE_INCHES / 2));
    // Deceleration is the same as acceleration here, which is the braking
    // case - and Drivetrain does set BRAKE on all four drive motors, so a
    // released stick really should stop the robot rather than let it drift.
    public static final double LINEAR_ACCEL_IPS2 = 220.0;
    public static final double TURN_ACCEL_DPS2 = 900.0;

    /** Field coordinates in inches, origin at the bottom-left wall. */
    public double xInches = FIELD_INCHES / 2;
    public double yInches = FIELD_INCHES / 2;

    /** Degrees clockwise from "up the field". */
    public double headingDeg;

    /** Robot-frame components recovered from the wheels, -1..1 each. */
    public double forwardCmd;
    public double strafeCmd;
    public double turnCmd;

    /** True while the robot is pinned against a wall, so the view can say so. */
    public boolean againstWall;

    private double vxIps;
    private double vyIps;
    private double omegaDps;

    public void reset() {
        xInches = FIELD_INCHES / 2;
        yInches = FIELD_INCHES / 2;
        headingDeg = 0;
        vxIps = 0;
        vyIps = 0;
        omegaDps = 0;
        forwardCmd = 0;
        strafeCmd = 0;
        turnCmd = 0;
        againstWall = false;
    }

    /** Sets the robot's simulated starting position without changing its heading. */
    public void setStartingPosition(double x, double y) {
        double half = ROBOT_INCHES / 2.0;
        xInches = Math.max(half, Math.min(FIELD_INCHES - half, x));
        yInches = Math.max(half, Math.min(FIELD_INCHES - half, y));
        vxIps = 0;
        vyIps = 0;
        omegaDps = 0;
        forwardCmd = 0;
        strafeCmd = 0;
        turnCmd = 0;
        againstWall = false;
    }

    /** Puts the robot at (x, y) facing {@code headingDeg}, stopped. */
    public void setStartingPose(double x, double y, double headingDeg) {
        this.headingDeg = wrap(headingDeg);
        setStartingPosition(x, y);
    }

    /** Field-frame velocity, inches per second, for things the robot pushes. */
    public double vxIps() { return vxIps; }
    public double vyIps() { return vyIps; }
    /** Turn rate, degrees per second clockwise. */
    public double omegaDps() { return omegaDps; }

    /**
     * Something solid stopped the robot moving along the field direction
     * (nx, ny), a unit vector pointing into the obstacle: drop that part of
     * the velocity, as the walls do, so it does not stick and then leap away.
     */
    public void blockedAlong(double nx, double ny) {
        double into = vxIps * nx + vyIps * ny;
        if (into > 0) {
            vxIps -= into * nx;
            vyIps -= into * ny;
        }
    }

    /** Half the robot's width along the field axes at its current heading. */
    public double halfExtent() {
        double h = Math.toRadians(headingDeg);
        return ROBOT_INCHES / 2 * (Math.abs(Math.cos(h)) + Math.abs(Math.sin(h)));
    }

    /** Advances the picture by {@code dtSec}. */
    public void step(double flPower, double frPower, double rlPower, double rrPower, double dtSec) {

        /*
         * The right-hand motors are reversed in the robot configuration, not by
         * setDirection() in Drivetrain, so a command of (+,-,+,-) is the robot
         * driving straight forward. Undo that mirror to get four wheel speeds
         * that all mean "forward" when positive. DrivetrainMotorTest pins this
         * convention down.
         */
        double fl = flPower;
        double fr = -frPower;
        double rl = rlPower;
        double rr = -rrPower;

        forwardCmd = (fl + fr + rl + rr) / 4.0;
        strafeCmd = (fl - fr - rl + rr) / 4.0;
        turnCmd = (fl - fr + rl - rr) / 4.0;

        // Robot-frame target velocities, then into field coordinates.
        double h = Math.toRadians(headingDeg);
        double strafe = strafeCmd * STRAFE_FRACTION;
        double targetVx = (strafe * Math.cos(h) + forwardCmd * Math.sin(h)) * MAX_SPEED_IPS;
        double targetVy = (forwardCmd * Math.cos(h) - strafe * Math.sin(h)) * MAX_SPEED_IPS;
        double targetOmega = turnCmd * MAX_TURN_DPS;

        // Ramp rather than step, or the robot teleports on a trigger press and
        // you cannot see which way it went.
        vxIps = approach(vxIps, targetVx, LINEAR_ACCEL_IPS2 * dtSec);
        vyIps = approach(vyIps, targetVy, LINEAR_ACCEL_IPS2 * dtSec);
        omegaDps = approach(omegaDps, targetOmega, TURN_ACCEL_DPS2 * dtSec);

        xInches += vxIps * dtSec;
        yInches += vyIps * dtSec;
        headingDeg = wrap(headingDeg + omegaDps * dtSec);

        // Stop at the walls, and drop the velocity so the robot does not stick
        // to them and then leap away when you reverse.
        double half = halfExtent();
        againstWall = false;
        if (xInches < half) { xInches = half; vxIps = 0; againstWall = true; }
        if (xInches > FIELD_INCHES - half) { xInches = FIELD_INCHES - half; vxIps = 0; againstWall = true; }
        if (yInches < half) { yInches = half; vyIps = 0; againstWall = true; }
        if (yInches > FIELD_INCHES - half) { yInches = FIELD_INCHES - half; vyIps = 0; againstWall = true; }
    }

    /**
     * Bearing from the robot to the point (x, y), degrees clockwise off the
     * nose. The launcher is fixed to the chassis, so this is the whole of the
     * aiming problem: a shot goes where the robot is pointing.
     */
    public double bearingTo(double x, double y) {
        return wrap(Math.toDegrees(Math.atan2(x - xInches, y - yInches)) - headingDeg);
    }

    private static double approach(double current, double target, double maxStep) {
        double delta = target - current;
        if (Math.abs(delta) <= maxStep) return target;
        return current + Math.signum(delta) * maxStep;
    }

    private static double wrap(double deg) {
        double d = deg % 360.0;
        if (d > 180.0) d -= 360.0;
        if (d < -180.0) d += 360.0;
        return d;
    }
}

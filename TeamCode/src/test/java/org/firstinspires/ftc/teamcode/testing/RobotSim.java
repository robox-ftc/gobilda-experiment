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

    /** FTC field, wall to wall. */
    public static final double FIELD_INCHES = 144.0;

    /** Rough footprint of this robot, for drawing only. */
    public static final double ROBOT_INCHES = 17.0;

    /*
     * Somewhere to aim at. The position is a GUESS - it is not measured off a
     * DECODE field - and it exists so the chassis has something to point at, not
     * so anyone can read a range off the screen. Move it and nothing but the
     * picture changes.
     */
    public static final double GOAL_X_INCHES = 12.0;
    public static final double GOAL_Y_INCHES = 132.0;

    /*
     * Speeds a goBILDA 5202 mecanum drive of about this size does on a good
     * battery. Nothing here was measured on the real robot - they set the scale
     * of the picture, not its correctness. If the robot on screen feels wrong
     * next to the real one, these are the numbers to change.
     */
    public static final double MAX_SPEED_IPS = 60.0;
    public static final double MAX_TURN_DPS = 220.0;
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
        double targetVx = (strafeCmd * Math.cos(h) + forwardCmd * Math.sin(h)) * MAX_SPEED_IPS;
        double targetVy = (forwardCmd * Math.cos(h) - strafeCmd * Math.sin(h)) * MAX_SPEED_IPS;
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
        double half = ROBOT_INCHES / 2;
        againstWall = false;
        if (xInches < half) { xInches = half; vxIps = 0; againstWall = true; }
        if (xInches > FIELD_INCHES - half) { xInches = FIELD_INCHES - half; vxIps = 0; againstWall = true; }
        if (yInches < half) { yInches = half; vyIps = 0; againstWall = true; }
        if (yInches > FIELD_INCHES - half) { yInches = FIELD_INCHES - half; vyIps = 0; againstWall = true; }
    }

    /**
     * Bearing from the robot to the goal, degrees clockwise off the nose.
     *
     * <p>The launcher is fixed to the chassis, so this is the whole of the aiming
     * problem: a shot goes where the robot is pointing.
     */
    public double bearingToGoal() {
        double dx = GOAL_X_INCHES - xInches;
        double dy = GOAL_Y_INCHES - yInches;
        return wrap(Math.toDegrees(Math.atan2(dx, dy)) - headingDeg);
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

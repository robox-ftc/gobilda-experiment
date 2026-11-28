package org.firstinspires.ftc.teamcode.devices;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * How this robot's goBILDA Pinpoint is fitted, and its readings in the
 * robot's own terms: forward, left and counterclockwise all positive, from
 * where it was last zeroed. Use {@link #forwardInches}, {@link #leftInches}
 * and {@link #headingDegrees} rather than the driver's getPosX/getPosY/
 * getHeading, which are in the Pinpoint's terms.
 *
 * <p>Check it with the "Cal 2: Pinpoint Check" OpMode, which says what to
 * change here.
 */
public final class PinpointSetup {

    /**
     * True on this robot: the Pinpoint reads as a mirror image of how the
     * robot moves. Its X port's pod counts the robot moving RIGHT, its Y
     * port's pod counts it moving BACKWARD, and its heading goes up when the
     * robot turns CLOCKWISE (the board is upside down).
     *
     * <p>Found with Cal 2 on 2026-11-20 (pushed 36 in forward: Y -39.9; 36 in
     * left: X -48.7; 90 degrees counterclockwise: heading -78.4), and the same
     * on 2026-11-18 in NewAuto (PINPOINT_UPSIDE_DOWN). A mirror image is still
     * self-consistent - the device turns its pod counts into a position with
     * a heading that is mirrored the same way - so the readings only need
     * mirroring back: forward = -Y, left = -X, heading = -heading.
     *
     * <p>Set false if the Pinpoint is remounted label side up with the
     * forward-measuring pod in the X port, then run Cal 2 again.
     */
    public static final boolean MIRRORED = true;

    /**
     * How the device counts each port, before the mirroring above is undone.
     * Cal 2 says if one must be flipped.
     */
    public static final EncoderDirection X_POD_DIRECTION = EncoderDirection.FORWARD;
    public static final EncoderDirection Y_POD_DIRECTION = EncoderDirection.FORWARD;

    /**
     * Where the pods are, from the centre of the robot. NOT YET MEASURED:
     * Cal 2 estimates them by turning on the spot; check with a ruler.
     */
    public static final double FORWARD_POD_LEFT_INCHES = 0;     // the pod that measures forward motion: inches left of centre (right is negative)
    public static final double STRAFE_POD_FORWARD_INCHES = 7.5; // the pod that measures sideways motion: inches forward of centre (behind is negative)

    private PinpointSetup() {}

    /** Sets the Pinpoint up as fitted. Does not zero it or calibrate the gyro. */
    public static void configure(GoBildaPinpointDriver pinpoint) {
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(X_POD_DIRECTION, Y_POD_DIRECTION);
        double[] offsets = deviceOffsets(MIRRORED, FORWARD_POD_LEFT_INCHES, STRAFE_POD_FORWARD_INCHES);
        pinpoint.setOffsets(offsets[0], offsets[1], DistanceUnit.INCH);
    }

    /**
     * The device's setOffsets(x, y) for pods at these places on the robot.
     * The device wants how far its X pod sits towards its +Y, and its Y pod
     * towards its +X. Mirrored, its X pod is the sideways one and its +Y is
     * the robot's backward, and its Y pod is the forward one and its +X is the
     * robot's right - so both come out negated and swapped.
     */
    public static double[] deviceOffsets(boolean mirrored, double forwardPodLeftInches, double strafePodForwardInches) {
        return mirrored
                ? new double[]{-strafePodForwardInches, -forwardPodLeftInches}
                : new double[]{forwardPodLeftInches, strafePodForwardInches};
    }

    /** Inches forward, as of the last update(). */
    public static double forwardInches(GoBildaPinpointDriver pinpoint) {
        return MIRRORED ? -pinpoint.getPosY(DistanceUnit.INCH) : pinpoint.getPosX(DistanceUnit.INCH);
    }

    /** Inches left, as of the last update(). */
    public static double leftInches(GoBildaPinpointDriver pinpoint) {
        return MIRRORED ? -pinpoint.getPosX(DistanceUnit.INCH) : pinpoint.getPosY(DistanceUnit.INCH);
    }

    /** Degrees counterclockwise, -180 to 180, as of the last update(). */
    public static double headingDegrees(GoBildaPinpointDriver pinpoint) {
        double heading = pinpoint.getHeading(AngleUnit.DEGREES);
        return MIRRORED ? AngleUnit.normalizeDegrees(-heading) : heading;
    }
}

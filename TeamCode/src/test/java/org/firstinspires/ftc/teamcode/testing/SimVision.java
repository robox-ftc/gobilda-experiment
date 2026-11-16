package org.firstinspires.ftc.teamcode.testing;

import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.testing.BiobuzzField.Alliance;
import org.firstinspires.ftc.teamcode.testing.BiobuzzField.End;
import org.firstinspires.ftc.teamcode.testing.BiobuzzField.V3;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseRaw;

import java.util.ArrayList;
import java.util.Collections;
import java.util.EnumMap;
import java.util.HashSet;
import java.util.Set;

/**
 * What the robot's webcam would report, worked out from where the robot is.
 *
 * <p>The SDK's BIOBUZZ library groups each CELL's four tags into a cluster, so
 * {@code getDetections()} returns one {@link AprilTagClusterDetection} per CELL
 * in view, posed at the CELL opening's center. This builds exactly that: the
 * cluster's pose in the camera frame (OpenCV axes: x right, y down, z out of
 * the lens), converted to {@link AprilTagPoseFtc} with the SDK's own formula
 * ({@code AprilTagProcessorImpl.rawPoseToFtcPose}). A cluster is in view when
 * at least one of its tags is in front of the lens, inside the field of view,
 * within range, and facing the camera.
 *
 * <p>The camera's place on the robot is whatever the OpMode passed to
 * {@code AprilTagProcessor.Builder.setCameraPose}, read the SDK's way: with
 * yaw, pitch and roll all 0 the lens points straight up, x to the robot's
 * right; pitch -90 points it forward. An OpMode that never calls it gets the
 * SDK's default, a camera at the robot's center looking up.
 *
 * <p>Not modelled: lens distortion, blur, lighting, occlusion by the HIVE or
 * the robot, latency, and noise. {@code robotPose} is left null - the BIOBUZZ
 * clusters move, so the SDK's field pose for them means nothing anyway.
 */
public final class SimVision {

    /** A typical FTC webcam's field of view at the SDK's default 640x480. An assumption. */
    public static final double HFOV_DEG = 70, VFOV_DEG = 52;
    /** How far away a 3.25 in. tag still decodes. An assumption. */
    public static final double MAX_RANGE = 120;
    /** How far off square-on a tag can be and still decode. An assumption. */
    public static final double MAX_VIEW_ANGLE_DEG = 70;

    private static AprilTagLibrary library;

    private volatile Position mountPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
    private volatile YawPitchRollAngles mountOrientation =
            new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0);

    private volatile boolean posed;
    private volatile double x, y, headingDeg;
    private final EnumMap<Alliance, End> up = new EnumMap<Alliance, End>(Alliance.class);
    private volatile Set<Integer> visible = Collections.emptySet();

    public SimVision() {
        for (Alliance a : Alliance.values()) up.put(a, BiobuzzField.upAtStart(a));
    }

    /** From the OpMode's {@code setCameraPose}. */
    public void setMount(Position position, YawPitchRollAngles orientation) {
        if (position != null) mountPosition = position;
        if (orientation != null) mountOrientation = orientation;
    }

    /** Where the robot is now and how the HIVES are tipped. */
    public synchronized void update(RobotSim sim, FieldSim field) {
        x = sim.xInches;
        y = sim.yInches;
        headingDeg = sim.headingDeg;
        if (field != null) {
            for (Alliance a : Alliance.values()) up.put(a, field.up(a));
        }
        posed = true;
    }

    /** Tag IDs the last {@link #detections()} saw. */
    public Set<Integer> visibleTags() {
        return visible;
    }

    /** What {@code AprilTagProcessor.getDetections()} returns. Empty until {@link #update}. */
    public synchronized ArrayList<AprilTagDetection> detections() {
        ArrayList<AprilTagDetection> out = new ArrayList<AprilTagDetection>();
        Set<Integer> seen = new HashSet<Integer>();
        if (!posed) {
            visible = seen;
            return out;
        }
        V3[] cam = cameraFrame();
        V3 eye = cam[3];
        double tanH = Math.tan(Math.toRadians(HFOV_DEG / 2)), tanV = Math.tan(Math.toRadians(VFOV_DEG / 2));
        double cosView = Math.cos(Math.toRadians(MAX_VIEW_ANGLE_DEG));

        for (Alliance a : Alliance.values()) {
            End upEnd = up.get(a);
            for (End cell : End.values()) {
                int first = BiobuzzField.firstTagId(a, cell);
                V3[] axes = BiobuzzField.clusterAxes(cell, upEnd);
                int found = 0;
                for (int i = 0; i < 4; i++) {
                    V3 tag = BiobuzzField.tagCenter(a, cell, upEnd, i);
                    V3 rel = tag.minus(eye);
                    double cx = rel.dot(cam[0]), cy = rel.dot(cam[1]), cz = rel.dot(cam[2]);
                    double range = rel.length();
                    boolean inView = cz > 1 && Math.abs(cx / cz) <= tanH && Math.abs(cy / cz) <= tanV
                            && range <= MAX_RANGE
                            // The tag's z points into it, so a camera that sees its face looks along +z.
                            && rel.dot(axes[2]) / range >= cosView;
                    if (inView) {
                        found++;
                        seen.add(first + i);
                    }
                }
                if (found > 0) out.add(clusterDetection(a, cell, upEnd, first, found, cam));
            }
        }
        visible = seen;
        return out;
    }

    private AprilTagClusterDetection clusterDetection(Alliance a, End cell, End upEnd, int firstId,
                                                      int found, V3[] cam) {
        V3 origin = BiobuzzField.opening(a, cell, upEnd);
        V3[] axes = BiobuzzField.clusterAxes(cell, upEnd);
        V3 rel = origin.minus(cam[3]);
        double tx = rel.dot(cam[0]), ty = rel.dot(cam[1]), tz = rel.dot(cam[2]);

        // R takes cluster coordinates to camera coordinates: its columns are the
        // cluster's axes seen from the camera.
        GeneralMatrixF r = new GeneralMatrixF(3, 3);
        for (int col = 0; col < 3; col++) {
            for (int row = 0; row < 3; row++) {
                r.put(row, col, (float) axes[col].dot(cam[row]));
            }
        }
        AprilTagPoseRaw raw = new AprilTagPoseRaw(tx, ty, tz, r);

        // AprilTagProcessorImpl.rawPoseToFtcPose, with the default inch/degree units.
        Orientation rot = Orientation.getOrientation(r, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
        AprilTagPoseFtc ftc = new AprilTagPoseFtc(
                tx, tz, -ty,
                -rot.firstAngle, rot.secondAngle, rot.thirdAngle,
                Math.hypot(tx, tz),
                Math.toDegrees(Math.atan2(-tx, tz)),
                Math.toDegrees(Math.atan2(-ty, tz)));

        int percent = Math.round(found * 100f / 4);
        return new AprilTagClusterDetection(percent, library().lookupCluster(firstId), DistanceUnit.INCH,
                ftc, raw, null, System.nanoTime());
    }

    private static synchronized AprilTagLibrary library() {
        if (library == null) library = AprilTagGameDatabase.getBioBuzzTagLibrary();
        return library;
    }

    /**
     * The camera's axes in the field - x (image right), y (image down), z (out
     * of the lens) - and, last, its position.
     */
    public synchronized V3[] cameraFrame() {
        double h = Math.toRadians(headingDeg);
        V3 right = new V3(Math.cos(h), -Math.sin(h), 0);
        V3 forward = new V3(Math.sin(h), Math.cos(h), 0);
        V3 upward = new V3(0, 0, 1);

        // AprilTagProcessor.Builder.build(): the camera's rotation on the robot
        // is intrinsic Z-X-Z by (yaw, pitch, roll); its columns are the camera's
        // axes in robot coordinates (x right, y forward, z up).
        YawPitchRollAngles o = mountOrientation;
        OpenGLMatrix m = new Orientation(AxesReference.INTRINSIC, AxesOrder.ZXZ, AngleUnit.DEGREES,
                (float) o.getYaw(AngleUnit.DEGREES), (float) o.getPitch(AngleUnit.DEGREES),
                (float) o.getRoll(AngleUnit.DEGREES), 0).getRotationMatrix();
        V3[] axes = new V3[4];
        for (int k = 0; k < 3; k++) {
            axes[k] = right.times(m.get(0, k)).plus(forward.times(m.get(1, k))).plus(upward.times(m.get(2, k)));
        }
        Position p = mountPosition.toUnit(DistanceUnit.INCH);
        axes[3] = new V3(x, y, 0).plus(right.times(p.x)).plus(forward.times(p.y)).plus(upward.times(p.z));
        return axes;
    }
}

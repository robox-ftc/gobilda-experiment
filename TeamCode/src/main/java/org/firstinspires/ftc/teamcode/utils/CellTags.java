package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

/**
 * Which AprilTag detections belong to our alliance's CELLs.
 *
 * SDK 12.0's BIOBUZZ library groups the four tags under each CELL into a cluster
 * (RED SCORING 30-33, RED AUDIENCE 34-37, BLUE AUDIENCE 38-41, BLUE SCORING 42-45),
 * reported as one AprilTagClusterDetection whose pose is the centre of the CELL
 * opening. A tag can still come back on its own as an AprilTagSingleDetection, so
 * both are accepted: red CELL tags are 30-37, blue 38-45 (Competition Manual 9.9).
 */
public final class CellTags {
    private CellTags() {}

    public static boolean isOurs(AprilTagDetection detection, boolean blue) {
        if (detection instanceof AprilTagClusterDetection) {
            AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
            return cluster.metadata != null && cluster.metadata.name != null
                    && cluster.metadata.name.startsWith(blue ? "BLUE" : "RED");
        }
        if (detection instanceof AprilTagSingleDetection) {
            AprilTagSingleDetection single = (AprilTagSingleDetection) detection;
            if (single.metadata == null) return false;
            return blue ? single.id >= 38 && single.id <= 45
                        : single.id >= 30 && single.id <= 37;
        }
        return false;
    }
}

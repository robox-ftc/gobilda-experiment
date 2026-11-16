package org.firstinspires.ftc.teamcode.testing;

import static org.mockito.Mockito.withSettings;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.mockito.Answers;
import org.mockito.MockedConstruction;
import org.mockito.Mockito;
import org.mockito.invocation.InvocationOnMock;

import java.util.ArrayList;

/**
 * A webcam that never sees an AprilTag.
 *
 * <p>The real {@link AprilTagProcessor} allocates OpenCV matrices, and OpenCV's
 * native library is built for the Control Hub, not the PC. Loading it here
 * throws {@code UnsatisfiedLinkError} - an Error, so an OpMode's
 * {@code catch (Exception e)} around its camera setup does not catch it, and the
 * OpMode dies in INIT. While {@link #install()} is in effect, the AprilTag
 * builder hands back a processor that reports no detections, so camera code
 * runs and simply finds nothing to aim at.
 *
 * <p>The portal's builder returns null. {@link VisionPortal} cannot be loaded
 * on the PC at all - its static initializer asks Android for the app context -
 * so not even a stand-in can exist. TeamCode reads tags through the processor
 * and null-checks the portal, so this is enough.
 *
 * <p>Like {@link SimClock}, this is Mockito's {@code mockConstruction}, so it
 * covers construction on the installing thread only.
 */
public final class SimCamera implements AutoCloseable {

    private final MockedConstruction<AprilTagProcessor.Builder> processorBuilders;
    private final MockedConstruction<VisionPortal.Builder> portalBuilders;

    private SimCamera() {
        final AprilTagProcessor processor = Mockito.mock(AprilTagProcessor.class,
                invocation -> invocation.getMethod().getName().equals("getDetections")
                        || invocation.getMethod().getName().equals("getFreshDetections")
                        ? new ArrayList<AprilTagDetection>()
                        : Answers.RETURNS_DEFAULTS.answer(invocation));

        processorBuilders = Mockito.mockConstruction(AprilTagProcessor.Builder.class,
                context -> withSettings().defaultAnswer(invocation -> builder(invocation, processor)));
        try {
            portalBuilders = Mockito.mockConstruction(VisionPortal.Builder.class,
                    context -> withSettings().defaultAnswer(invocation -> builder(invocation, null)));
        } catch (RuntimeException | Error e) {
            processorBuilders.close(); // a mockConstruction left open breaks the thread's next one
            throw e;
        }
    }

    /** Swaps the vision builders for stand-ins until the returned scope is closed. */
    public static SimCamera install() {
        return new SimCamera();
    }

    /** Builder setters return the builder, as the real ones do; build() returns the stand-in. */
    private static Object builder(InvocationOnMock invocation, Object built) throws Throwable {
        if (invocation.getMethod().getName().equals("build")) return built;
        return Answers.RETURNS_SELF.answer(invocation);
    }

    @Override
    public void close() {
        processorBuilders.close();
        portalBuilders.close();
    }
}

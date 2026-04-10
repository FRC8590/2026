package lib.woodsonrobotics.vision;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Generic class for a camera on the robot. This is thread-safe.
 */
public abstract class Camera {
    public record Resolution(int width, int height) {
    };

    /**
     * Name of the camera.
     */
    protected String name;

    /**
     * Transform of the camera rotation and translation relative to the center of
     * the robot.
     */
    protected Transform3d robotToCamTransform;

    /**
     * Height x width resolution of the camera.
     */
    protected Resolution cameraResolution;

    /**
     * Layout of the FRC field, usually taken from
     * {@link AprilTagFields#loadField}.
     */
    protected AprilTagFieldLayout fieldLayout;

    /**
     * 
     * @param name                The name of the camera.
     * @param fieldLayout         The AprilTag field layout, as returned by
     *                            {@link AprilTagFields#loadField}.
     * @param robotToCamTransform The {@link Transform3d} object representing the
     *                            position of the camera relative to the robot.
     * @param cameraResolution    The resolution of the camera.
     */
    public Camera(String name, AprilTagFieldLayout fieldLayout, Transform3d robotToCamTransform,
            Resolution cameraResolution) {
        this.name = name;
        this.fieldLayout = fieldLayout;
        this.robotToCamTransform = robotToCamTransform;
        this.cameraResolution = cameraResolution;
    }

    /**
     * Estimated pose of the camera relative to the field layout.
     * 
     * This is atomic and refreshes the results.
     * 
     * @return The estimated pose of the camera.
     */
    public abstract Optional<EstimatedPose> getEstimatedGlobalPose();

    /**
     * Get the standard deviations from the camera.
     * 
     * We don't really know what this does; it's basically some math
     * stuff that's important for YagSL.
     */
    public abstract Matrix<N3, N1> getStandardDeviations();

    /**
     * Get the list of targets currently tracked by the camera.
     * 
     * This is atomic and refreshes the results.
     */
    public abstract List<TrackedAprilTag> getAprilTags();

    /**
     * Refresh the results of the camera.
     * 
     * This is atomic.
     */
    public abstract void refresh();

    /**
     * @return Transform of the camera rotation and translation relative to the
     *         center of the robot.
     */
    public final Transform3d getRobotToCamera() {
        return robotToCamTransform;
    }

    /**
     * @return Layout of the FRC field.
     */
    public final AprilTagFieldLayout getFieldLayout() {
        return fieldLayout;
    }

    /**
     * @return The name of the camera.
     */
    public final String getName() {
        return name;
    }

    /**
     * @return The resolution of the camera.
     */
    public final Resolution getResolution() {
        return cameraResolution;
    }
}

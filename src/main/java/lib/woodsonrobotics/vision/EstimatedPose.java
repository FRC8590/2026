package lib.woodsonrobotics.vision;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Estimated pose on a field based on vision.
 */
public interface EstimatedPose {
    /**
     * Get the estimated pose as a Pose2d object.
     * 
     * @return Pose2d object with the pose.
     */
    public Pose2d toPose2d();

    /**
     * The timestamp when this was recorded.
     * 
     * @return The timestamp, in seconds.
     */
    public double getTimestampSeconds();
}

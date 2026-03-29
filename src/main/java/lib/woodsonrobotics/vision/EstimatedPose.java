package lib.woodsonrobotics.vision;

import edu.wpi.first.math.geometry.Pose2d;

/* Estimated pose on a field based on vision. */
public interface EstimatedPose {
    /* The estimated pose as a Pose2d object. */
    public Pose2d toPose2d();

    /* The timestamp when this was recorded, in seconds. */
    public double getTimestampSeconds();
}

package lib.woodsonrobotics.vision;

import edu.wpi.first.math.geometry.Transform3d;

/**
 * An AprilTag tracked by a camera.
 */
public interface TrackedAprilTag {
    /**
     * Get the transform that maps camera space (X = forward, Y = left, Z = up) to
     * object/fiducial tag space (X forward, Y left, Z up).
     */
    public Transform3d getDistance();

    /**
     * Get the yaw of the AprilTag.
     * 
     * @return The yaw, in radians.
     */
    public double getYaw();

    /**
     * Get the pitch of the AprilTag.
     * 
     * @return The pitch, in radians.
     */
    public double getPitch();

    /**
     * Get the ID of this AprilTag.
     * 
     * @return The AprilTag ID.
     */
    public int getId();
}

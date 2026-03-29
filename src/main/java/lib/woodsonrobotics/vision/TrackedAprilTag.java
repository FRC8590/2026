package lib.woodsonrobotics.vision;

import edu.wpi.first.math.geometry.Transform3d;

/* An AprilTag tracked by a camera. */
public interface TrackedAprilTag {
    /*
     * Get the transform that maps camera space (X = forward, Y = left, Z = up) to
     * object/fiducial tag space (X forward, Y left, Z up).
     */
    public Transform3d getDistance();

    public double getYaw();

    public double getPitch();

    /* Get the ID of this AprilTag. */
    public int getId();
}

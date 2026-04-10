package lib.woodsonrobotics.vision.photon;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import lib.woodsonrobotics.vision.TrackedAprilTag;

public class PhotonAprilTag implements TrackedAprilTag {
    private PhotonTrackedTarget trackedTarget;

    public PhotonAprilTag(PhotonTrackedTarget target) {
        trackedTarget = target;
    }

    @Override
    public int getId() {
        return trackedTarget.getFiducialId();
    }

    @Override
    public Transform3d getDistance() {
        return trackedTarget.getBestCameraToTarget();
    }

    @Override
    public double getPitch() {
        return trackedTarget.getPitch();
    }

    @Override
    public double getYaw() {
        return trackedTarget.getYaw();
    }
}

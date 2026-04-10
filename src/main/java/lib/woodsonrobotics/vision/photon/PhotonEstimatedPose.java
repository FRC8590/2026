package lib.woodsonrobotics.vision.photon;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;
import lib.woodsonrobotics.vision.EstimatedPose;

public final class PhotonEstimatedPose implements EstimatedPose {
    private final EstimatedRobotPose estimatedRobotPose;

    public PhotonEstimatedPose(EstimatedRobotPose pose) {
        estimatedRobotPose = pose;
    }

    @Override
    public Pose2d toPose2d() {
        return estimatedRobotPose.estimatedPose.toPose2d();
    }

    @Override
    public double getTimestampSeconds() {
        return estimatedRobotPose.timestampSeconds;
    }
}

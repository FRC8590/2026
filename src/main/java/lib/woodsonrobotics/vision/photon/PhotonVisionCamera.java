package lib.woodsonrobotics.vision.photon;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import lib.woodsonrobotics.vision.Camera;
import lib.woodsonrobotics.vision.EstimatedPose;
import lib.woodsonrobotics.vision.TrackedAprilTag;

public class PhotonVisionCamera extends Camera {
    protected final PhotonCamera camera;
    /**
     * Pose estimator for camera.
     */
    private final PhotonPoseEstimator poseEstimator;
    /**
     * Standard Deviation for single tag readings for pose estimation.
     */
    private final Matrix<N3, N1> singleTagStdDevs;
    /**
     * Standard deviation for multi-tag readings for pose estimation.
     */
    private final Matrix<N3, N1> multiTagStdDevs;

    /**
     * Current standard deviations used.
     */
    private Matrix<N3, N1> currentStdDevs;

    /**
     * Results list to be updated periodically and cached to avoid unnecessary
     * queries.
     */
    protected List<PhotonPipelineResult> resultsList = new ArrayList<>();

    /* Lock used to synchronized accesses to the results list. */
    protected Lock resultsLock = new ReentrantLock();

    public PhotonVisionCamera(String name, AprilTagFieldLayout fieldLayout, Transform3d robotToCamTransform,
            Resolution cameraResolution) {
        super(name, fieldLayout, robotToCamTransform, cameraResolution);
        camera = new PhotonCamera(name);
        poseEstimator = new PhotonPoseEstimator(fieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                robotToCamTransform);
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        singleTagStdDevs = VecBuilder.fill(2, 2, 4);
        multiTagStdDevs = VecBuilder.fill(0.05, 0.05, 0.1);
    }

    public static PhotonVisionCamera newArduCamera(String name, AprilTagFieldLayout fieldLayout,
            Transform3d robotToCamTransform) {
        return new PhotonVisionCamera(name, fieldLayout, robotToCamTransform, new Resolution(640, 480));
    }

    @Override
    public Matrix<N3, N1> getStandardDeviations() {
        return currentStdDevs;
    }

    /**
     * Get the estimated robot pose. Updates the current robot pose estimation
     * and standard deviations.
     * *
     * 
     * @return Estimated pose.
     */
    @Override
    public Optional<EstimatedPose> getEstimatedGlobalPose() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        resultsLock.lock();
        try {
            refresh();
            for (var change : resultsList) {
                visionEst = poseEstimator.update(change);
                updateEstimationStdDevs(visionEst, change.getTargets());
            }
            if (visionEst.isPresent()) {
                return Optional.of(new PhotonEstimatedPose(visionEst.get()));
            }
            resultsList.clear();
        } finally {
            resultsLock.unlock();
        }

        return Optional.empty();
    }

    /**
     * Update the latest results.
     */
    @Override
    public void refresh() {
        resultsLock.lock();
        try {
            resultsList.addAll(camera.getAllUnreadResults());
            resultsList.sort((PhotonPipelineResult a, PhotonPipelineResult b) -> {
                return a.getTimestampSeconds() >= b.getTimestampSeconds() ? 1 : -1;
            });
        } finally {
            resultsLock.unlock();
        }
    }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates
     * dynamic standard deviations based
     * on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets       All targets in this camera frame
     */
    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            currentStdDevs = singleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = singleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an
            // average-distance metric
            for (var tgt : targets) {
                var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) {
                    continue;
                }
                numTags++;
                avgDist += tagPose
                        .get()
                        .toPose2d()
                        .getTranslation()
                        .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                currentStdDevs = singleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) {
                    estStdDevs = multiTagStdDevs;
                }
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4) {
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                } else {
                    estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                }
                currentStdDevs = estStdDevs;
            }
        }
    }

    @Override
    public List<TrackedAprilTag> getAprilTags() {
        var targets = new ArrayList<TrackedAprilTag>();
        resultsLock.lock();
        try {
            refresh();
            for (PhotonPipelineResult result : resultsList) {
                if (result.hasTargets()) {
                    for (PhotonTrackedTarget target : result.getTargets()) {
                        targets.add(new PhotonAprilTag(target));
                    }
                }
            }
            resultsList.clear();
        } finally {
            resultsLock.unlock();
        }
        return targets;
    }
}

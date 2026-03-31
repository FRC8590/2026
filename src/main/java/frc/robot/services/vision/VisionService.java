package frc.robot.services.vision;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import lib.woodsonrobotics.telemetry.notify.DriveNotifier;
import lib.woodsonrobotics.vision.Camera;
import lib.woodsonrobotics.vision.EstimatedPose;
import lib.woodsonrobotics.vision.TrackedAprilTag;
import swervelib.SwerveDrive;

public class VisionService {
    public final List<Camera> allCameras;

    public VisionService(List<Camera> cameras) {
        allCameras = cameras;
    }

    public VisionService(Camera[] camerasList) {
        allCameras = Arrays.asList(camerasList);
    }

    /* Start the refresh thread. */
    public void startVisionThread() {
        new Thread(() -> {
            while (true) {
                for (Camera camera : allCameras) {
                    camera.refresh();
                }

                try {
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    DriveNotifier.internalError("startVisionThread", "Vision thread was interrupted");
                }
            }
        }).start();
    }

    /**
     * Update the pose estimation inside of {@link SwerveDrive} with all of the
     * given poses.
     *
     * @param swerveDrive {@link SwerveDrive} instance.
     */
    public void updateSwerveEstimation(SwerveDrive swerveDrive) {
        for (Camera camera : allCameras) {
            Optional<EstimatedPose> estimatedPose = camera.getEstimatedGlobalPose();

            if (estimatedPose.isPresent()) {
                var pose = estimatedPose.get();
                swerveDrive.addVisionMeasurement(pose.toPose2d(),
                        pose.getTimestampSeconds(),
                        camera.getStandardDeviations());
            }
        }
    }

    /**
     * Gets a pose estimate using a single AprilTag based on tx/ty angles and
     * distance.
     * This is more stable for precision alignment than multi-tag pose estimation.
     * 
     * @param tagId       The ID of the AprilTag to use
     * @param camera      The camera that sees the tag
     * @param currentPose Current pose from the pose estimator using wheel odometry,
     *                    as returned by {@link SwerveDrive#getPose()}
     * @return Optional containing the estimated robot pose, or empty if the tag
     *         isn't visible
     */
    public Optional<Pose2d> getSingleTagPoseEstimate(int tagId, Camera camera, Pose2d currentPose) {
        // Check if the camera can see the tag
        Optional<TrackedAprilTag> targetOpt = Optional.empty();
        for (TrackedAprilTag tag : camera.getAprilTags()) {
            if (tag.getId() == tagId) {
                targetOpt = Optional.of(tag);
                break;
            }
        }

        if (targetOpt.isEmpty()) {
            return Optional.empty();
        }

        TrackedAprilTag target = targetOpt.get();

        // Get the tag's position in the field
        Optional<Pose3d> tagPoseOpt = camera.getFieldLayout().getTagPose(tagId);
        if (tagPoseOpt.isEmpty()) {
            return Optional.empty();
        }
        Pose3d tagPose = tagPoseOpt.get();

        // Get camera position relative to robot
        Transform3d robotToCamera = camera.getRobotToCamera();

        // Calculate horizontal angle (tx) and vertical angle (ty)
        double tx = target.getYaw();
        double ty = target.getPitch();

        // Get 3D distance from target
        // This is more reliable than the ambiguous pose from SolvePNP
        double distance3d = target.getDistance().getTranslation().getNorm();

        // Calculate 2D distance (projected to the ground plane)
        // Use camera mounting angle and ty
        double cameraPitchRadians = robotToCamera.getRotation().getY();
        double distance2d = distance3d * Math.cos(Math.toRadians(ty) + cameraPitchRadians);

        // Calculate robot angle relative to tag
        Rotation2d robotAngle = currentPose.getRotation();
        Rotation2d cameraAngle = Rotation2d.fromDegrees(robotToCamera.getRotation().getZ());
        Rotation2d totalAngle = robotAngle.plus(cameraAngle);

        // Calculate angle to tag
        Rotation2d tagAngle = totalAngle.plus(Rotation2d.fromDegrees(-tx));

        // Calculate robot position
        // Start at tag position, move distance at the calculated angle, and account for
        // camera offset
        Translation2d tagTranslation = tagPose.toPose2d().getTranslation();
        Translation2d cameraTranslation = tagTranslation.plus(
                new Translation2d(
                        -distance2d * Math.cos(tagAngle.getRadians()),
                        -distance2d * Math.sin(tagAngle.getRadians())));

        // Adjust for camera position on robot
        Translation2d robotTranslation = cameraTranslation.minus(
                new Translation2d(
                        robotToCamera.getX() * Math.cos(robotAngle.getRadians()) -
                                robotToCamera.getY() * Math.sin(robotAngle.getRadians()),
                        robotToCamera.getX() * Math.sin(robotAngle.getRadians()) +
                                robotToCamera.getY() * Math.cos(robotAngle.getRadians())));

        // Return robot pose with original rotation (we use the gyro for rotation)
        return Optional.of(new Pose2d(robotTranslation, robotAngle));
    }

    /**
     * Gets best single-tag pose estimate from all cameras.
     * Prioritizes closer tags for better accuracy.
     * 
     * @param tagId       The ID of the AprilTag to use
     * @param currentPose Current pose from the pose estimator using wheel odometry,
     *                    as returned by {@link SwerveDrive#getPose()}
     * @return Optional containing the estimated robot pose, or empty if the tag
     *         isn't visible
     */
    public Optional<Pose2d> getBestSingleTagPoseEstimate(int tagId, Pose2d currentPose) {
        Optional<Pose2d> bestPose = Optional.empty();
        double bestDistance = Double.MAX_VALUE;

        int usedId = -1;
        for (Camera camera : allCameras) {
            // Get the target from this camera if it exists
            Optional<TrackedAprilTag> targetOpt = Optional.empty();
            for (TrackedAprilTag tag : camera.getAprilTags()) {
                int foundTagId = tag.getId();
                if (tagId == foundTagId) {
                    double distance = tag.getDistance().getTranslation().getNorm();
                    // Only process if this is closer than the current best
                    if (distance < bestDistance) {
                        bestDistance = distance;
                        targetOpt = Optional.of(tag);
                        usedId = tagId;
                    }
                    break;
                }
            }

            if (targetOpt.isPresent()) {
                Optional<Pose2d> pose = getSingleTagPoseEstimate(usedId, camera, currentPose);
                if (pose.isPresent()) {
                    bestPose = pose;
                }
            }
        }

        return bestPose;
    }

    /**
     * Gets the field-relative pose of a given AprilTag from the layout.
     * Use this for navigation goals.
     */
    public Optional<Pose2d> getTagFieldPose(int tagId) {
        for (Camera camera : allCameras) {
            Optional<Pose3d> tagPose = camera.getFieldLayout().getTagPose(tagId);
            if (tagPose.isPresent()) {
                return Optional.of(tagPose.get().toPose2d());
            }
        }
        return Optional.empty();
    }
}

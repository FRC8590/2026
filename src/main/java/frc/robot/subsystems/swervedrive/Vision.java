package frc.robot.subsystems.swervedrive;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import swervelib.SwerveDrive;

/**
 * Example PhotonVision class to aid in the pursuit of accurate odometry. Taken
 * from
 * https://gitlab.com/ironclad_code/ironclad-2024/-/blob/master/src/main/java/frc/robot/vision/Vision.java?ref_type=heads
 */
public class Vision {

  /**
   * April Tag Field Layout of the year.
   */
  public static final AprilTagFieldLayout fieldLayout = Constants.layout;
  /**
   * Ambiguity defined as a value between (0,1). Used in
   * {@link Vision#filterPose}.
   */
  // Riley: Never used?
  //private final double maximumAmbiguity = 0.15;

  /**
   * Count of times that the odom thinks we're more than 10meters away from the
   * april tag.
   */
  // Riley: also never used?
  // private double longDistangePoseEstimationCount = 0;
  /**
   * Current pose from the pose estimator using wheel odometry.
   */
  private Supplier<Pose2d> currentPose;
  /**
   * Field from {@link swervelib.SwerveDrive#field}
   */
  private Field2d field2d;

  public EstimatedRobotPose estimatedVisionPose;

  /**
   * Constructor for the Vision class.
   *
   * @param currentPose Current pose supplier, should reference
   *                    {@link SwerveDrive#getPose()}
   * @param field       Current field, should be {@link SwerveDrive#field}
   */
  public Vision(Supplier<Pose2d> currentPose, Field2d field) {
    this.currentPose = currentPose;
    this.field2d = field;
  }

  /**
   * Calculates a target pose relative to an AprilTag on the field.
   *
   * @param aprilTag    The ID of the AprilTag.
   * @param robotOffset The offset {@link Transform2d} of the robot to apply to
   *                    the pose for the robot to position
   *                    itself correctly.
   * @return The target pose of the AprilTag.
   */
  public static Pose2d getAprilTagPose(int aprilTag, Transform2d robotOffset) {
    Optional<Pose3d> aprilTagPose3d = fieldLayout.getTagPose(aprilTag);
    if (aprilTagPose3d.isPresent()) {
      return aprilTagPose3d.get().toPose2d().transformBy(robotOffset);
    } else {
      throw new RuntimeException("Cannot get AprilTag " + aprilTag + " from field " + fieldLayout.toString());
    }

  }

  /**
   * Calculates whether it has the apriltag in view
   *
   * @param number The numberof the AprilTag.
   * @return Whether it has that number in view of any camera.
   */

  public static boolean seesNumber(int number) {
    for (Cameras c : Cameras.values()) {
      if (!c.resultsList.isEmpty()) {
        for (PhotonPipelineResult result : c.resultsList) {
          for (PhotonTrackedTarget target : result.getTargets()) {
            if (target.getFiducialId() == number) {
              return true;
            }
          }
        }
      }
    }
    return false;
  }

  /**
   * Update the pose estimation inside of {@link SwerveDrive} with all of the
   * given poses.
   *
   * @param swerveDrive {@link SwerveDrive} instance.
   */
  public void updatePoseEstimation(SwerveDrive swerveDrive) {
    for (Cameras camera : Cameras.values()) {
      Optional<EstimatedRobotPose> poseEst = getEstimatedGlobalPose(camera);

      if (poseEst != null && poseEst.isPresent()) {

        estimatedVisionPose = poseEst.get();

        var pose = poseEst.get();
        swerveDrive.addVisionMeasurement(pose.estimatedPose.toPose2d(),
            pose.timestampSeconds,
            camera.curStdDevs);
        // System.out.println("vision estimation added");
        SmartDashboard.putNumber("poseX", pose.estimatedPose.toPose2d().getX());
        SmartDashboard.putNumber("poseY", pose.estimatedPose.toPose2d().getY());

      }
    }

  }

  /**
   * Generates the estimated robot pose. Returns empty if:
   * <ul>
   * <li>No Pose Estimates could be generated</li>
   * <li>The generated pose estimate was considered not accurate</li>
   * </ul>
   *
   * @return an {@link EstimatedRobotPose} with an estimated pose, timestamp, and
   *         targets used to create the estimate
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Cameras camera) {
    Optional<EstimatedRobotPose> poseEst = camera.getEstimatedGlobalPose();
    return poseEst;
  }

  /**
   * Get distance of the robot from the AprilTag pose.
   *
   * @param id AprilTag ID
   * @return Distance
   */
  public double getDistanceFromAprilTag(int id) {
    Optional<Pose3d> tag = fieldLayout.getTagPose(id);
    return tag.map(pose3d -> PhotonUtils.getDistanceToPose(currentPose.get(), pose3d.toPose2d())).orElse(-1.0);
  }

  /**
   * Get tracked target from a camera of AprilTagID
   *
   * @param id     AprilTag ID
   * @param camera Camera to check.
   * @return Tracked target.
   */
  public PhotonTrackedTarget getTargetFromId(int id, Cameras camera) {
    PhotonTrackedTarget target = null;
    for (PhotonPipelineResult result : camera.resultsList) {
      if (result.hasTargets()) {
        for (PhotonTrackedTarget i : result.getTargets()) {
          if (i.getFiducialId() == id) {
            return i;
          }
        }
      }
    }
    return target;

  }

  /**
   * Update the {@link Field2d} to include tracked targets/
   */
  public void updateVisionField() {

    List<PhotonTrackedTarget> targets = new ArrayList<PhotonTrackedTarget>();
    for (Cameras c : Cameras.values()) {
      if (!c.resultsList.isEmpty()) {
        PhotonPipelineResult latest = c.resultsList.get(0);
        if (latest.hasTargets()) {
          targets.addAll(latest.targets);
        }
      }
    }

    List<Pose2d> poses = new ArrayList<>();
    for (PhotonTrackedTarget target : targets) {
      if (fieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
        Pose2d targetPose = fieldLayout.getTagPose(target.getFiducialId()).get().toPose2d();
        poses.add(targetPose);
      }
    }

    field2d.getObject("tracked targets").setPoses(poses);
  }

  /**
   * Camera Enum to select each camera
   */
  public enum Cameras {

    /**
     *
     */

    // This is where the cameras are created, with their name and and position
    // relative to the robot's center
    RIGHT_CAM("right",
        new Rotation3d(0, 0, Units.degreesToRadians(0)),
        new Translation3d(Units.inchesToMeters(5),
            Units.inchesToMeters(-1.5),
            Units.inchesToMeters(12.5)),
        VecBuilder.fill(4, 4, 8), VecBuilder.fill(0.5, 0.5, 1)),
    LEFT_CAM("left",
        new Rotation3d(0, 0, Units.degreesToRadians(0)),
        new Translation3d(Units.inchesToMeters(5.5),
            Units.inchesToMeters(11),
            Units.inchesToMeters(12.5)),
        VecBuilder.fill(4, 4, 8), VecBuilder.fill(0.5, 0.5, 1));

    /**
     * Latency alert to use when high latency is detected.
     */
    public final Alert latencyAlert;
    /**
     * Camera instance for comms.
     */
    public PhotonCamera camera;
    /**
     * Pose estimator for camera.
     */
    public final PhotonPoseEstimator poseEstimator;
    /**
     * Standard Deviation for single tag readings for pose estimation.
     */
    private final Matrix<N3, N1> singleTagStdDevs;
    /**
     * Standard deviation for multi-tag readings for pose estimation.
     */
    private final Matrix<N3, N1> multiTagStdDevs;
    /**
     * Transform of the camera rotation and translation relative to the center of
     * the robot
     */
    private final Transform3d robotToCamTransform;
    /**
     * Current standard deviations used.
     */
    public Matrix<N3, N1> curStdDevs;
    /**
     * Estimated robot pose.
     */
    public Optional<EstimatedRobotPose> estimatedRobotPose;
    /**
     * Simulated camera instance which only exists during simulations.
     */
    public PhotonCameraSim cameraSim;
    /**
     * Results list to be updated periodically and cached to avoid unnecessary
     * queries.
     */
    public List<PhotonPipelineResult> resultsList = new ArrayList<>();
    /**
     * Last read from the camera timestamp to prevent lag due to slow data fetches.
     */
    private double lastReadTimestamp = Microseconds.of(NetworkTablesJNI.now()).in(Seconds);

    private boolean hasSetOffset = false;

    private double timerOffset = 0;

    /**
     * Construct a Photon Camera class with help. Standard deviations are fake
     * values, experiment and determine
     * estimation noise on an actual robot.
     *
     * @param name                  Name of the PhotonVision camera found in the PV
     *                              UI.
     * @param robotToCamRotation    {@link Rotation3d} of the camera.
     * @param robotToCamTranslation {@link Translation3d} relative to the center of
     *                              the robot.
     * @param singleTagStdDevs      Single AprilTag standard deviations of estimated
     *                              poses from the camera.
     * @param multiTagStdDevsMatrix Multi AprilTag standard deviations of estimated
     *                              poses from the camera.
     */
    Cameras(String name, Rotation3d robotToCamRotation, Translation3d robotToCamTranslation,
        Matrix<N3, N1> singleTagStdDevs, Matrix<N3, N1> multiTagStdDevsMatrix) {
      latencyAlert = new Alert("'" + name + "' Camera is experiencing high latency.", AlertType.kWarning);

      camera = null;
      try {
        camera = new PhotonCamera(name);
      } catch (Exception e) {
        System.out.println(e);
      }

      // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
      robotToCamTransform = new Transform3d(robotToCamTranslation, robotToCamRotation);

      poseEstimator = new PhotonPoseEstimator(Vision.fieldLayout,
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          robotToCamTransform);
      poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

      this.singleTagStdDevs = singleTagStdDevs;
      this.multiTagStdDevs = multiTagStdDevsMatrix;
    }

    /**
     * Get the result with the least ambiguity from the best tracked target within
     * the Cache. This may not be the most
     * recent result!
     *
     * @return The result in the cache with the least ambiguous best tracked target.
     *         This is not the most recent result!
     */
    public Optional<PhotonPipelineResult> getBestResult() {
      if (resultsList.isEmpty()) {
        return Optional.empty();
      }

      PhotonPipelineResult bestResult = resultsList.get(0);
      double amiguity = bestResult.getBestTarget().getPoseAmbiguity();
      double currentAmbiguity = 0;
      for (PhotonPipelineResult result : resultsList) {
        currentAmbiguity = result.getBestTarget().getPoseAmbiguity();
        if (currentAmbiguity < amiguity && currentAmbiguity > 0) {
          bestResult = result;
          amiguity = currentAmbiguity;
        }
      }
      return Optional.of(bestResult);
    }

    /**
     * Get the estimated robot pose. Updates the current robot pose estimation,
     * standard deviations, and flushes the
     * cache of results.
     *
     * @return Estimated pose.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
      updateUnreadResults();
      return estimatedRobotPose;
    }

    /**
     * Update the latest results, cached with a maximum refresh rate of 1req/15ms.
     * Sorts the list by timestamp.
     */
    private void updateUnreadResults() {

      double mostRecentTimestamp;

      if (resultsList.isEmpty()) {
        mostRecentTimestamp = 0.0;
      } else {
        mostRecentTimestamp = resultsList.get(0).getTimestampSeconds() - timerOffset;
      }

      // System.out.println(mostRecentTimestamp);
      double currentTimestamp = Microseconds.of(NetworkTablesJNI.now()).in(Seconds);
      double debounceTime = Milliseconds.of(15).in(Seconds);

      if (hasSetOffset == false && mostRecentTimestamp != 0) {
        timerOffset = mostRecentTimestamp;
        hasSetOffset = true;
      }

      for (PhotonPipelineResult result : resultsList) {
        mostRecentTimestamp = Math.max(mostRecentTimestamp, result.getTimestampSeconds() - timerOffset);
      }

      if ((resultsList.isEmpty() || (currentTimestamp - mostRecentTimestamp >= debounceTime)) &&
          (currentTimestamp - lastReadTimestamp) >= debounceTime) {
        resultsList = Robot.isReal() ? camera.getAllUnreadResults() : cameraSim.getCamera().getAllUnreadResults();
        lastReadTimestamp = currentTimestamp;
        resultsList.sort((PhotonPipelineResult a, PhotonPipelineResult b) -> {
          return a.getTimestampSeconds() >= b.getTimestampSeconds() ? 1 : -1;
        });
        if (!resultsList.isEmpty()) {
          updateEstimatedGlobalPose();
        }
      }

    }

    /**
     * The latest estimated robot pose on the field from vision data. This may be
     * empty. This should only be called once
     * per loop.
     *
     * <p>
     * Also includes updates for the standard deviations, which can (optionally) be
     * retrieved with
     * {@link Cameras#updateEstimationStdDevs}
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate
     *         timestamp, and targets used for
     *         estimation.
     */
    private void updateEstimatedGlobalPose() {
      Optional<EstimatedRobotPose> visionEst = Optional.empty();
      for (var change : resultsList) {
        visionEst = poseEstimator.update(change);
        updateEstimationStdDevs(visionEst, change.getTargets());
      }
      estimatedRobotPose = visionEst;
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
        curStdDevs = singleTagStdDevs;

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
          curStdDevs = singleTagStdDevs;
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
          curStdDevs = estStdDevs;
        }
      }
    }

  }

  /**
   * Gets a pose estimate using a single AprilTag based on tx/ty angles and
   * distance.
   * This is more stable for precision alignment than multi-tag pose estimation.
   * 
   * @param tagId  The ID of the AprilTag to use
   * @param camera The camera that sees the tag
   * @return Optional containing the estimated robot pose, or empty if the tag
   *         isn't visible
   */
  public Optional<Pose2d> getSingleTagPoseEstimate(int tagId, Cameras camera) {
    // Check if the camera can see the tag
    Optional<PhotonTrackedTarget> targetOpt = Optional.empty();
    for (PhotonPipelineResult result : camera.resultsList) {
      if (result.hasTargets()) {
        for (PhotonTrackedTarget target : result.getTargets()) {
          if (target.getFiducialId() == tagId) {
            targetOpt = Optional.of(target);
            break;
          }
        }
      }
    }

    if (targetOpt.isEmpty()) {
      return Optional.empty();
    }

    PhotonTrackedTarget target = targetOpt.get();

    // Get the tag's position in the field
    Optional<Pose3d> tagPoseOpt = fieldLayout.getTagPose(tagId);
    if (tagPoseOpt.isEmpty()) {
      return Optional.empty();
    }
    Pose3d tagPose = tagPoseOpt.get();

    // Get camera position relative to robot
    Transform3d robotToCamera = camera.robotToCamTransform;

    // Calculate horizontal angle (tx) and vertical angle (ty)
    double tx = target.getYaw();
    double ty = target.getPitch();

    // Get 3D distance from target
    // This is more reliable than the ambiguous pose from SolvePNP
    double distance3d = target.getBestCameraToTarget().getTranslation().getNorm();

    // Calculate 2D distance (projected to the ground plane)
    // Use camera mounting angle and ty
    double cameraPitchRadians = robotToCamera.getRotation().getY();
    double distance2d = distance3d * Math.cos(Math.toRadians(ty) + cameraPitchRadians);

    // Calculate robot angle relative to tag
    Rotation2d robotAngle = currentPose.get().getRotation();
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
   * @param tagId The ID of the AprilTag to use
   * @return Optional containing the estimated robot pose, or empty if the tag
   *         isn't visible
   */
  public Optional<Pose2d> getBestDoubleTagPoseEstimate(int firstTagId, int secondTagId) {
    Optional<Pose2d> bestPose = Optional.empty();
    double bestDistance = Double.MAX_VALUE;

    int usedId = -1;
    for (Cameras camera : Cameras.values()) {
      // Get the target from this camera if it exists
      Optional<PhotonTrackedTarget> targetOpt = Optional.empty();
      for (PhotonPipelineResult result : camera.resultsList) {
        if (result.hasTargets()) {
          for (PhotonTrackedTarget target : result.getTargets()) {
            int tagId = target.getFiducialId();
            if (tagId == firstTagId || tagId == secondTagId) {
              double distance = target.getBestCameraToTarget().getTranslation().getNorm();
              // Only process if this is closer than the current best
              if (distance < bestDistance) {
                bestDistance = distance;
                targetOpt = Optional.of(target);
                usedId = tagId;
              }
              // break;
            }
          }
        }
      }

      if (targetOpt.isPresent()) {
        Optional<Pose2d> pose = getSingleTagPoseEstimate(usedId, camera);
        if (pose.isPresent()) {
          bestPose = pose;
        }
      }
    }

    return bestPose;
  }

  public Optional<Pose2d> getBestSingleTagPoseEstimate(int tagId) {
    // -1 will never be a valid tag
    return getBestDoubleTagPoseEstimate(tagId, -1);
  }
}

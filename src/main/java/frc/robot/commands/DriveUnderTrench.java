package frc.robot.commands;

import java.util.Arrays;
import java.util.stream.IntStream;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.services.vision.VisionService;
import frc.robot.subsystems.drive.Swerve;
import lib.woodsonrobotics.SystemWrapper;

/**
 * Drives the robot through the nearest trench.
 */
public class DriveUnderTrench extends Command {

    // How far past the tag to go (meters)
    private static final double EXIT_OFFSET_METERS = -3.0;

    // How far past *before* the tag to go before going through it
    // (in meters). This prevents us from getting stuck on the bump.
    private static final double ALIGN_OFFSET_METERS = 1.5;

    // Speed of the robot for driving (in meters per second)
    private static final double MAX_VELOCITY = 6.0;

    // Trench entry tags (facing alliance zone)
    private static final int[] RED_ENTRY_TAG_IDS = { 7, 12 };
    private static final int[] BLUE_ENTRY_TAG_IDS = { 23, 28 };

    // Trench exit tags (facing neutral zone)
    private static final int[] RED_EXIT_TAG_IDS = { 6, 1 };
    private static final int[] BLUE_EXIT_TAG_IDS = { 17, 22 };

    private final int[] RED_TAG_IDS = IntStream
            .concat(Arrays.stream(RED_ENTRY_TAG_IDS), Arrays.stream(RED_EXIT_TAG_IDS))
            .toArray();
    private final int[] BLUE_TAG_IDS = IntStream
            .concat(Arrays.stream(BLUE_ENTRY_TAG_IDS), Arrays.stream(BLUE_EXIT_TAG_IDS))
            .toArray();

    private final SystemWrapper<? extends Swerve> driveSystem;
    private final VisionService visionService;
    private Command pathfindCommand;

    public DriveUnderTrench(SystemWrapper<? extends Swerve> drive, VisionService vision) {
        driveSystem = drive;
        visionService = vision;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        var driveOpt = driveSystem.get();
        if (driveOpt.isEmpty()) {
            return;
        }
        var drive = driveOpt.get();

        Pose2d robotPose = drive.getPose();
        int[] tags = RobotContainer.isRedAlliance() ? RED_TAG_IDS : BLUE_TAG_IDS;

        // Find the nearest tag
        NearestTag nearestTag = findNearestTag(tags, robotPose);

        var targetTagPose = visionService.getTagFieldPose(nearestTag.tagId);
        Pose2d goalPose = offsetAlongFacing(targetTagPose, EXIT_OFFSET_METERS);
        Command driveThroughCommand = AutoBuilder.pathfindToPose(
                goalPose,
                new PathConstraints(MAX_VELOCITY, MAX_VELOCITY,
                        Units.degreesToRadians(270), Units.degreesToRadians(360)),
                0.0);

        if (nearestTag.distance < 1.0) {
            // If we're closer then 1m, then there's no need to align.
            pathfindCommand = driveThroughCommand;
        } else {

            // First, we pathfind to right before the trench, which lets us stay
            // in line with it. Then, our pathfind towards the trench will go straight
            // through. This prevents us from getting stuck on the bump. We could
            // avoid this by using a navmesh in PathPlanner, but I don't know how
            // to do that. Maybe Riley can figure it out.
            Pose2d alignPose = offsetAlongFacing(targetTagPose, ALIGN_OFFSET_METERS);

            Command alignCommand = AutoBuilder.pathfindToPose(
                    alignPose,
                    new PathConstraints(MAX_VELOCITY, MAX_VELOCITY,
                            Units.degreesToRadians(270), Units.degreesToRadians(360)),
                    MAX_VELOCITY); // Nonzero end velocity so it flows into the next command
            pathfindCommand = alignCommand.andThen(driveThroughCommand);
        }

        pathfindCommand.initialize();
    }

    @Override
    public void execute() {
        if (pathfindCommand != null)
            pathfindCommand.execute();
    }

    @Override
    public boolean isFinished() {
        return pathfindCommand == null || pathfindCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        if (pathfindCommand != null)
            pathfindCommand.end(interrupted);
    }

    private record NearestTag(int tagId, double distance) {
    };

    /**
     * Find the nearest trench tag.
     */
    private NearestTag findNearestTag(int[] tagIds, Pose2d robotPose) {
        double bestDistance = Double.MAX_VALUE;
        int bestTag = -1;

        for (int tagId : tagIds) {
            var pose = visionService.getTagFieldPose(tagId);
            double distance = robotPose.getTranslation()
                    .getDistance(pose.getTranslation());
            if (distance < bestDistance) {
                bestDistance = distance;
                bestTag = tagId;
            }
        }

        assert (bestTag != -1);
        return new NearestTag(bestTag, bestDistance);
    }

    /**
     * Computes a goal pose offset from a tag pose along the tag's facing direction.
     * The robot heading is set to match the direction of travel (same as tag
     * facing).
     */
    private Pose2d offsetAlongFacing(Pose2d tagPose, double offsetMeters) {
        Rotation2d facing = tagPose.getRotation();
        Translation2d offset = new Translation2d(
                offsetMeters * Math.cos(facing.getRadians()),
                offsetMeters * Math.sin(facing.getRadians()));
        return new Pose2d(tagPose.getTranslation().plus(offset), facing);
    }
}
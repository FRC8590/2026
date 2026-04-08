package frc.robot.commands;

import java.util.Arrays;
import java.util.Optional;
import java.util.stream.IntStream;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.services.vision.VisionService;
import frc.robot.subsystems.drive.Swerve;
import lib.woodsonrobotics.PathfinderCommand;
import lib.woodsonrobotics.SystemWrapper;

/**
 * Drives the robot through the nearest trench.
 */
public class DriveUnderTrench extends SequentialCommandGroup {

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
    public static final int[] RED_EXIT_TAG_IDS = { 1, 6 };
    public static final int[] BLUE_EXIT_TAG_IDS = { 17, 22 };

    private final int[] RED_TAG_IDS = IntStream
            .concat(Arrays.stream(RED_ENTRY_TAG_IDS), Arrays.stream(RED_EXIT_TAG_IDS))
            .toArray();
    private final int[] BLUE_TAG_IDS = IntStream
            .concat(Arrays.stream(BLUE_ENTRY_TAG_IDS), Arrays.stream(BLUE_EXIT_TAG_IDS))
            .toArray();

    private abstract class TrenchPathfinderCommand extends PathfinderCommand {
        private final SystemWrapper<? extends Swerve> driveSystem;
        private final VisionService visionService;

        public TrenchPathfinderCommand(SystemWrapper<? extends Swerve> drive, VisionService vision) {
            super(MAX_VELOCITY, 5.0);
            visionService = vision;
            driveSystem = drive;
            addRequirements(drive);
        }

        protected Optional<VisionService.NearestTag> findNearestTag() {
            var driveOpt = driveSystem.get();
            if (driveOpt.isEmpty()) {
                return Optional.empty();
            }
            var drive = driveOpt.get();

            Pose2d robotPose = drive.getPose();
            int[] tags = RobotContainer.isRedAlliance() ? RED_TAG_IDS : BLUE_TAG_IDS;
            var nearestTag = visionService.findNearestTag(tags, robotPose);

            return Optional.of(nearestTag);
        }
    }

    private final class AlignWithTrench extends TrenchPathfinderCommand {
        public AlignWithTrench(SystemWrapper<? extends Swerve> drive, VisionService vision) {
            super(drive, vision);
        }

        @Override
        public void initialize() {
            var nearestTagOpt = findNearestTag();
            if (nearestTagOpt.isEmpty()) {
                return;
            }

            var nearestTag = nearestTagOpt.get();

            // If we're closer then 1m, then there's no need to align.
            if (nearestTag.distance() > 1.0) {
                Pose2d goalPose = PathfinderCommand.offsetAlong(nearestTag.tagPose(), ALIGN_OFFSET_METERS, 0);
                initializeWithGoalPose(goalPose);
            }
        }
    }

    private final class DriveThroughTrench extends TrenchPathfinderCommand {
        public DriveThroughTrench(SystemWrapper<? extends Swerve> drive, VisionService vision) {
            super(drive, vision);
        }

        @Override
        public void initialize() {
            var nearestTagOpt = findNearestTag();
            if (nearestTagOpt.isEmpty()) {
                return;
            }

            var nearestTag = nearestTagOpt.get();

            Pose2d goalPose = PathfinderCommand.offsetAlong(nearestTag.tagPose(), EXIT_OFFSET_METERS, 0);
            initializeWithGoalPose(goalPose);
        }
    }

    public DriveUnderTrench(SystemWrapper<? extends Swerve> drive, VisionService vision) {
        addCommands(
                // First, we pathfind to right before the trench, which lets us stay
                // in line with it. Then, our pathfind towards the trench will go straight
                // through. This prevents us from getting stuck on the bump. We could
                // avoid this by using a navmesh in PathPlanner, but I don't know how
                // to do that. Maybe Riley can figure it out.
                new AlignWithTrench(drive, vision),
                new DriveThroughTrench(drive, vision));
    }
}
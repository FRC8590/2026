package frc.robot.commands.intake;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.services.vision.VisionService;
import frc.robot.subsystems.drive.Swerve;
import lib.woodsonrobotics.PathfinderCommand;
import lib.woodsonrobotics.SystemWrapper;

/**
 * From the neutral zone, go to the nearest hub. This is where the majority of
 * the fuel lies!
 */
public class GoToHubFromNeutralZone extends PathfinderCommand {
    // IDs for AprilTags on the hub facing the neutral zone
    private static int[] NEUTRAL_ZONE_TAG_IDS = { 4, 3, 19, 20 };

    // How far before the tag to go (meters)
    private static final double HUB_OFFSET_METERS = 0.2;

    // Speed of the robot for driving (in meters per second)
    private static final double MAX_VELOCITY = 6.0;

    private final SystemWrapper<? extends Swerve> driveSystem;
    private final VisionService visionService;

    public GoToHubFromNeutralZone(SystemWrapper<? extends Swerve> drive, VisionService vision) {
        super(MAX_VELOCITY, 5.0);
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
        var nearestTag = visionService.findNearestTag(NEUTRAL_ZONE_TAG_IDS, robotPose);
        var goalPose = PathfinderCommand.offsetAlongFacing(nearestTag.tagPose(), HUB_OFFSET_METERS);
        initializeWithGoalPose(goalPose);
    }
}

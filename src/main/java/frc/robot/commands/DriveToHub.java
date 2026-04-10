package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.services.vision.VisionService;
import lib.woodsonrobotics.PathfinderCommand;

/**
 * Drive directly to the hub from our alliance zone.
 */
public class DriveToHub extends PathfinderCommand {
    // IDs for AprilTags on the hub facing the neutral zone
    private static int RED_TAG_ID = 10;
    private static int BLUE_TAG_ID = 26;

    // How far before the tag to go (meters)
    // TODO: This needs measuring
    private static final double HUB_OFFSET_METERS = 1.7;

    // Speed of the robot for driving (in meters per second)
    private static final double MAX_VELOCITY = 6.0;

    private final VisionService visionService;

    public DriveToHub(VisionService vision) {
        super(MAX_VELOCITY, 5.0);
        visionService = vision;
    }

    @Override
    public void initialize() {
        int tagId = RobotContainer.isRedAlliance() ? RED_TAG_ID : BLUE_TAG_ID;
        var targetPose = visionService.getTagFieldPose(tagId);
        var goalPose = PathfinderCommand.offsetAlongFacing(targetPose, HUB_OFFSET_METERS);
        initializeWithGoalPose(goalPose);
    }
}

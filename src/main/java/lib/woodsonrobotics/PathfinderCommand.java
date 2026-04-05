package lib.woodsonrobotics;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Simple wrapper around a command that pathfinds to a given pose using
 * PathPlanner.
 */
public abstract class PathfinderCommand extends Command {
    // Speed of the robot for driving (in meters per second)
    private final double maxVelocity;

    // End velocity
    private final double goalEndVelocity;

    private Command pathfindCommand;

    public PathfinderCommand(double maxVelocity, double goalEndVelocity) {
        this.maxVelocity = maxVelocity;
        this.goalEndVelocity = goalEndVelocity;
    }

    /**
     * Initialize the command with the target pose.
     */
    protected void initializeWithGoalPose(Pose2d goalPose) {
        // Find the nearest tag
        pathfindCommand = AutoBuilder.pathfindToPose(
                goalPose,
                new PathConstraints(maxVelocity, maxVelocity,
                        Units.degreesToRadians(270), Units.degreesToRadians(360)),
                goalEndVelocity);
        pathfindCommand.initialize();
    }

    @Override
    abstract public void initialize();

    @Override
    public final void execute() {
        if (pathfindCommand != null)
            pathfindCommand.execute();
    }

    @Override
    public final boolean isFinished() {
        return pathfindCommand == null || pathfindCommand.isFinished();
    }

    @Override
    public final void end(boolean interrupted) {
        if (pathfindCommand != null) {
            pathfindCommand.end(interrupted);
        }
    }

    /**
     * Computes a goal pose offset from a tag pose.
     * The robot heading is set to match the direction of travel (same as tag
     * facing + offsetDegrees).
     */
    public static Pose2d offsetAlong(Pose2d tagPose, double offsetMeters, double offsetDegrees) {
        Rotation2d facing = tagPose.getRotation();
        Translation2d offset = new Translation2d(
                offsetMeters * Math.cos(facing.getRadians()),
                offsetMeters * Math.sin(facing.getRadians()));
        Rotation2d robotHeading = facing.plus(Rotation2d.fromDegrees(offsetDegrees));
        return new Pose2d(tagPose.getTranslation().plus(offset), robotHeading);
    }

    /**
     * Computes a goal pose offset from a tag pose along the tag's facing direction.
     * The robot heading is set to match the direction of travel (same as tag
     * facing).
     */
    public static Pose2d offsetAlongFacing(Pose2d tagPose, double offsetMeters) {
        return offsetAlong(tagPose, offsetMeters, 180);
    }
}

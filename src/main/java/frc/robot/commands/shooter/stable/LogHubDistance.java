package frc.robot.commands.shooter.stable;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.services.vision.VisionService;
import frc.robot.subsystems.drive.Swerve;
import lib.woodsonrobotics.SystemWrapper;

/**
 * Log the distance to the hub.
 * 
 * This is only here because mech won't let us get an accurate model, so we
 * have to do it during matches. Ideally, once we have a good regression model,
 * we can remove this.
 */
public class LogHubDistance extends Command {
    SystemWrapper<? extends Swerve> driveSystem;
    VisionService visionService;

    public LogHubDistance(SystemWrapper<? extends Swerve> drive, VisionService vision) {
        driveSystem = drive;
        visionService = vision;
    }

    // We do this in initialize() to prevent the logs from being flooded
    // while the driver pushes the button.
    @Override
    public void initialize() {
        var driveOpt = driveSystem.get();
        if (driveOpt.isEmpty()) {
            return;
        }

        var drive = driveOpt.get();
        var tagPose = visionService.getTagFieldPose(RobotContainer.getHubAprilTag());
        double distanceMeters = drive.getPose().getTranslation()
                .getDistance(tagPose.getTranslation());
        System.out.println("Distance to hub: " + distanceMeters);
    }

    @Override
    public void execute() {
        // Do nothing
    }

    @Override
    public boolean isFinished() {
        // This command is always finished
        return true;
    }
}

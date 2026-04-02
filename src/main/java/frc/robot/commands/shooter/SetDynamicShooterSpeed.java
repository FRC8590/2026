package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.RobotContainer;
import frc.robot.services.vision.VisionService;
import lib.woodsonrobotics.SystemWrapper;

/**
 * Set the shooter speed based on the distance to the hub.
 * 
 * This does not run the feeder.
 */
public class SetDynamicShooterSpeed extends Command {
    private final SystemWrapper<Shooter> shooterSystem;
    private final SystemWrapper<? extends Swerve> driveSystem;
    private final VisionService visionService;

    public SetDynamicShooterSpeed(SystemWrapper<Shooter> shooter,
            SystemWrapper<? extends Swerve> drive, VisionService vision) {
        shooterSystem = shooter;
        driveSystem = drive;
        visionService = vision;
        // We intentionally don't add the drive system here, because we use
        // this in a parallel command group with AimAtTarget, which also
        // requires the drive system. It's fine here because we only need
        // the drive system for the distance to the hub, which doesn't
        // change while AimAtTarget is running.
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        var drive = driveSystem.get();
        if (!drive.isPresent()) {
            return;
        }

        var tagPose = visionService.getTagFieldPose(RobotContainer.getHubAprilTag());

        double distanceMeters = drive.get().getPose().getTranslation()
                .getDistance(tagPose.getTranslation());
        double rpm = Shooter.distanceToRPM(distanceMeters);
        shooterSystem.ifEnabled(shooter -> shooter.setGoalRPM(rpm));
    }

    @Override
    public boolean isFinished() {
        var shooter = shooterSystem.get();
        if (shooter.isEmpty()) {
            return true;
        }
        return shooter.get().atRPM();
    }
}

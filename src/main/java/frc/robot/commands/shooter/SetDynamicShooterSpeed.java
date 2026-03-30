package frc.robot.commands.shooter;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.services.vision.VisionService;

import lib.woodsonrobotics.SystemWrapper;

/* Set the shooter speed based on the distance to the hub.
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
        addRequirements(shooter, drive);
    }

    @Override
    public void execute() {
        var drive = driveSystem.get();
        if (!drive.isPresent()) {
            return;
        }

        int primaryId = 26;
        var tagPoseOpt = visionService.getTagFieldPose(primaryId);
        if (tagPoseOpt.isEmpty()) {
            return;
        }

        double distanceMeters = drive.get().getPose().getTranslation()
                .getDistance(tagPoseOpt.get().getTranslation());
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

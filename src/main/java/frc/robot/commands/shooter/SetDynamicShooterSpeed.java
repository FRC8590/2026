package frc.robot.commands.shooter;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.services.vision.VisionService;

import lib.woodsonrobotics.SystemWrapper;

/* Set the shooter speed based on the distance to the hub.
 * 
 * This stops the shooter when finished. This does not run
 * the feeder.
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
        var pose = drive.get().getPose();

        int primaryId = 26;// TODO: Vision.getHubAprilTag();
        var result = visionService.getBestSingleTagPoseEstimate(primaryId, pose);
        if (!result.isPresent()) {
            // Nothing seen!
            return;
        }

        /*
         * The model assumes the distance from getMeasureX() is
         * horizontal distance to the tag, which is generally
         * true, but not exact, depending on where the tag is
         * positioned relative to the hub center. If shots are
         * consistently off by a fixed amount at all distances,
         * we can correct it with a small offset constant.
         */
        double distanceMeters = result.get().getMeasureX().in(Units.Meters);
        double rpm = Shooter.distanceToRPM(distanceMeters);
        shooterSystem.ifEnabled(shooter -> shooter.setGoalRPM(rpm));
    }

    @Override
    public void end(boolean interrupted) {
        shooterSystem.ifEnabled(shooter -> shooter.setGoalRPM(0));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

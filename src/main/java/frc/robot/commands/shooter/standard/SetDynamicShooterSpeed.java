package frc.robot.commands.shooter.standard;

import java.util.ArrayList;

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

    // We add an arbitrary adjustment to the RPM toaccount for some
    // undershooting. This will need tuning.
    private static final double RPM_ADJUSTMENT = 400;

    // All the RPM values used during shooting.
    // This is so we can log an average at the end.
    private final ArrayList<Double> usedValues = new ArrayList<>();

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
        double rpm = Shooter.distanceToRPM(distanceMeters) + RPM_ADJUSTMENT;
        usedValues.add(rpm);
        shooterSystem.ifEnabled(shooter -> shooter.setGoalRPM(rpm));
    }

    @Override
    public void end(boolean interrupted) {
        double sum = usedValues.stream().mapToDouble(Double::doubleValue).sum();
        double average = sum / usedValues.size();
        System.out.println("Average RPM: " + average);
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

package frc.robot.commands.shooter.stable;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.services.vision.VisionService;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.shooter.Shooter;
import lib.woodsonrobotics.SystemWrapper;

/**
 * Set the shooter speed to a fixed RPM.
 * 
 * This does not run the feeder.
 */
public class SetStableShooterSpeed extends Command {
    private final SystemWrapper<Shooter> shooterSystem;
    private final SystemWrapper<? extends Swerve> driveSystem;
    private final VisionService visionService;

    GenericEntry rpmEntry = Shuffleboard
            .getTab("Console")
            .add("Stable RPM", 2000)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("Min", 0, "Max", Shooter.SHOOTER_MAX_RPM))
            .getEntry();

    private long lastProcessedTimestamp = 0;
    private double currentTargetRPM = 2000;
    private double distanceMeters = -1;

    public SetStableShooterSpeed(SystemWrapper<Shooter> shooter, SystemWrapper<? extends Swerve> drive,
            VisionService vision) {
        shooterSystem = shooter;
        driveSystem = drive;
        visionService = vision;
        addRequirements(shooter, drive);
    }

    @Override
    public void initialize() {
        var driveOpt = driveSystem.get();
        if (driveOpt.isEmpty()) {
            return;
        }

        // This is only here because mech won't let us get an accurate model, so we
        // have to do it during matches. Ideally, once we have a good regression model,
        // we can remove this.
        var drive = driveOpt.get();
        var tagPose = visionService.getTagFieldPose(RobotContainer.getHubAprilTag());
        distanceMeters = drive.getPose().getTranslation()
                .getDistance(tagPose.getTranslation());
        System.out.println("Distance to hub: " + distanceMeters);
    }

    @Override
    public void execute() {
        long currentTimestamp = rpmEntry.getLastChange();
        if (currentTimestamp > lastProcessedTimestamp) {
            lastProcessedTimestamp = currentTimestamp;
            currentTargetRPM = rpmEntry.getDouble(2000);
        }
        shooterSystem.ifEnabled(shooter -> shooter.setGoalRPM(currentTargetRPM));
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

package frc.robot.commands.shooter.stable;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import lib.woodsonrobotics.SystemWrapper;

/**
 * Set the shooter speed to a fixed RPM.
 * 
 * This does not run the feeder.
 */
public class SetStableShooterSpeed extends Command {
    private final SystemWrapper<Shooter> shooterSystem;

    GenericEntry rpmEntry = Shuffleboard
            .getTab("Console")
            .add("Stable RPM", 2000)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("Min", 0, "Max", Shooter.SHOOTER_MAX_RPM))
            .getEntry();

    private long lastProcessedTimestamp = 0;
    private double currentTargetRPM = 2000;

    public SetStableShooterSpeed(SystemWrapper<Shooter> shooter) {
        shooterSystem = shooter;
        addRequirements(shooter);
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

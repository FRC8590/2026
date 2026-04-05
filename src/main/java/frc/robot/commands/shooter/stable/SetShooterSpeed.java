package frc.robot.commands.shooter.stable;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import lib.woodsonrobotics.SystemWrapper;

/**
 * Set the shooter speed to a fixed RPM.
 * 
 * This does not run the feeder.
 */
public class SetShooterSpeed extends Command {
    private final SystemWrapper<Shooter> shooterSystem;
    private final int goalRPM;

    public SetShooterSpeed(SystemWrapper<Shooter> shooter, int rpm) {
        shooterSystem = shooter;
        goalRPM = rpm;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooterSystem.ifEnabled(shooter -> shooter.setGoalRPM(goalRPM));
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

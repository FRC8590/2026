package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

import lib.woodsonrobotics.SystemWrapper;

/* Set the shooter speed to a fixed RPM.
 * 
 * This stops the shooter when finished. This does not run the
 * feeder.
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
    public void end(boolean interrupted) {
        shooterSystem.ifEnabled(shooter -> shooter.setGoalRPM(0));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.Shooter;

public class StableShoot extends ParallelCommandGroup {
    public StableShoot(Shooter shooter, Belt belt) {
        addCommands(
                shooter.shooterSetStableGoalRPM(),
                belt.beltAndIndexerRun());
        addRequirements(shooter, belt);
    }

}
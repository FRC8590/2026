package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Indexer;

public class StableShoot extends SequentialCommandGroup {
    public StableShoot(Shooter shooter, Belt belt, Indexer indexer) {
        addCommands(
                shooter.shooterSetStableGoalRPM(),
                new WaitUntilCommand(shooter::atRPM),
                new Feed(belt, indexer));
        addRequirements(shooter);
    }

}
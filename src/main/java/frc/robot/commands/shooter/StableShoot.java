package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.feeder.Feed;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Indexer;

import lib.woodsonrobotics.SystemWrapper;

public class StableShoot extends SequentialCommandGroup {
    public StableShoot(SystemWrapper<Shooter> shooter,
            SystemWrapper<Belt> belt, SystemWrapper<Indexer> indexer) {
        addCommands(
                new SetShooterSpeed(shooter, 2000),
                new WaitUntilCommand(shooter.condition(Shooter::atRPM)),
                new Feed(belt, indexer));
    }

}
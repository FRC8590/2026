package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.feeder.Feed;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import lib.woodsonrobotics.SystemWrapper;

/*
 * Shoot fuel into the hub from a preset distance.
 * 
 * This does no alignment and always sets the shooter to a hardcoded
 * RPM, and will run the feeder when the shooter is up to speed.
 *
 * Both the shooter and feeder will be stopped when this is finished.
 */
public class StableShoot extends SequentialCommandGroup {
    public StableShoot(SystemWrapper<Shooter> shooter,
            SystemWrapper<Belt> belt, SystemWrapper<Indexer> indexer) {
        addCommands(
                new SetShooterSpeed(shooter, 2000),
                new WaitUntilCommand(shooter.condition(Shooter::atRPM)),
                new Feed(belt, indexer));
    }

}
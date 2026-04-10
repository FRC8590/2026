package frc.robot.commands.shooter.stable;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.feeder.Feed;
import frc.robot.services.vision.VisionService;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.feeder.Belt;
import frc.robot.subsystems.feeder.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import lib.woodsonrobotics.SystemWrapper;

/**
 * Shoot fuel into the hub from a preset distance.
 */
public class StableShoot extends SequentialCommandGroup {
    public StableShoot(SystemWrapper<Shooter> shooter,
            SystemWrapper<Belt> belt, SystemWrapper<Indexer> indexer, SystemWrapper<? extends Swerve> drive,
            VisionService vision) {
        addCommands(
                new SetStableShooterSpeed(shooter, drive, vision),
                new Feed(belt, indexer)
                        .finallyDo(() -> shooter.ifEnabled(shoot -> shoot.setGoalRPM(0))));
    }

}
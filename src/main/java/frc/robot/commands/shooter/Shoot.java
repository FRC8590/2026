package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.AimAtTarget;
import frc.robot.commands.feeder.Feed;
import frc.robot.services.vision.VisionService;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

import lib.woodsonrobotics.SystemWrapper;

public class Shoot extends SequentialCommandGroup {
    public Shoot(SystemWrapper<Shooter> shooter, SystemWrapper<Belt> belt,
            SystemWrapper<Indexer> indexer, VisionService vision, SystemWrapper<Swerve> drive) {
        addCommands(
                new AimAtTarget(vision, drive),
                new SetDynamicShooterSpeed(shooter, drive, vision),
                new WaitUntilCommand(shooter.condition(Shooter::atRPM)),
                new Feed(belt, indexer));
    }

}

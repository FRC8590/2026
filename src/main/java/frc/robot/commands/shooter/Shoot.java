package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.AimAtTarget;
import frc.robot.commands.feeder.Feed;
import frc.robot.services.vision.VisionService;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.shooter.Shooter;
import lib.woodsonrobotics.SystemWrapper;

/*
 * Shoot fuel into the hub.
 * 
 * This will first align the robot to the hub, then set the
 * shooter motor based on the distance to the hub (from vision),
 * and finally run the feeder once the shooter is up to speed.
 * 
 * Both the shooter and feeder will be stopped when this is finished.
 */
public class Shoot extends SequentialCommandGroup {
    public Shoot(SystemWrapper<Shooter> shooter, SystemWrapper<Belt> belt,
            SystemWrapper<Indexer> indexer, VisionService vision, SystemWrapper<? extends Swerve> drive) {
        addCommands(
                new PrintCommand("shoot 0"),
                new AimAtTarget(vision, drive),
                new PrintCommand("shoot 1"),
                new SetDynamicShooterSpeed(shooter, drive, vision),
                new PrintCommand("shoot 2"),
                new Feed(belt, indexer));
    }

}

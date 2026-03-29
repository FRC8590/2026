package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.services.vision.VisionService;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class Shoot extends SequentialCommandGroup {
    public Shoot(Shooter shooter, Belt belt, VisionService vision, Swerve drive) {
        addCommands(
                new AimAtTarget(vision, drive),
                shooter.shooterSetRPMFromVision(),
                new WaitUntilCommand(shooter::atRPM),
                belt.beltAndIndexerRun());
        addRequirements(shooter, belt);
    }

}

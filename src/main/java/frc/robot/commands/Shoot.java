package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.Shooter;

public class Shoot extends SequentialCommandGroup {
    public Shoot(Shooter shooter, Belt belt) {
        addCommands(shooter.shooterSetRPMFromVision(),
                new WaitUntilCommand(shooter::atRPM),
                belt.beltAndIndexerRun());
        addRequirements(shooter, belt);
    }

}

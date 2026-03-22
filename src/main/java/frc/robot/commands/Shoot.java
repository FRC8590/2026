package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;

public class Shoot extends SequentialCommandGroup {
    public Shoot() {
        addCommands(
                Constants.shooter.shooterSetRPMFromVision(),
                new WaitUntilCommand(Constants.shooter::atRPM),
                Constants.belt.beltAndIndexerRun());
    }

}

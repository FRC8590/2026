package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;

public class Shoot extends SequentialCommandGroup {
    public Shoot() {
        addCommands(
                Robot
                        .getInstance().m_robotContainer.shooter.shooterSetRPMFromVision(),
                new WaitUntilCommand(Robot
                        .getInstance().m_robotContainer.shooter::atRPM),
                Robot
                        .getInstance().m_robotContainer.belt.beltAndIndexerRun());
    }

}

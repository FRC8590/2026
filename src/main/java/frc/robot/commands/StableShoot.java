package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;

public class StableShoot extends ParallelCommandGroup {
    public StableShoot() {
        addCommands(
                Robot
                        .getInstance().m_robotContainer.shooter.shooterSetStableGoalRPM(),
                Robot
                        .getInstance().m_robotContainer.belt.beltAndIndexerRun());
        addRequirements(getRequirements());
    }

}
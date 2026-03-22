package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;

import frc.robot.Constants;

public class StableShoot extends SequentialCommandGroup {
    public StableShoot() {
        addCommands(
                Constants.shooter.shooterSetStableGoalRPM(),
                new WaitUntilCommand(Constants.shooter::atRPM),
                Constants.belt.beltAndIndexerRun());
    }

}

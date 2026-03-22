package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;

import frc.robot.Constants;

public class StableShoot extends ParallelCommandGroup {
    public StableShoot() {
        addCommands(
                Constants.shooter.shooterSetStableGoalRPM(),
                Constants.belt.beltAndIndexerRun());
        addRequirements(Constants.belt, Constants.shooter);
    }

}

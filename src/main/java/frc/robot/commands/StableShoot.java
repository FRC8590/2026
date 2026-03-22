package frc.robot.commands;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import frc.robot.Constants;


public class StableShoot extends ParallelCommandGroup {
    public StableShoot() {
        addCommands(
            Constants.shooter.shooterSetStableGoalRPM(),
            Constants.belt.beltAndIndexerRun()
        );
        addRequirements(getRequirements());
    }
    
}
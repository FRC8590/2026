package frc.robot.commands;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class StopShooter extends ParallelCommandGroup
{
    public StopShooter()
    {
        addCommands(
                Constants.shooter.stopShooter());

        addRequirements(getRequirements());
        
    }
}
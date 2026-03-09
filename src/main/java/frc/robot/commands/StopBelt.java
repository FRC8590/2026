package frc.robot.commands;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class StopBelt extends ParallelCommandGroup
{
    public StopBelt()
    {
        addCommands(
                Constants.belt.stopBelt());

        addRequirements(getRequirements());
        
    }
}
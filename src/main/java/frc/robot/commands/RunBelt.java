package frc.robot.commands;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class RunBelt extends ParallelCommandGroup
{
    public RunBelt()
    {
        addCommands(
            Constants.belt.runBelt());

        addRequirements(getRequirements());
    }

}

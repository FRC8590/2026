package frc.robot.commands;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
public class climbTest extends  SequentialCommandGroup
{
    public climbTest()
    {
        addCommands(
        Constants.climb.extend()
        );
        addRequirements(Constants.climb);
    }
}
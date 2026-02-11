package frc.robot.commands;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class IntakeTest extends SequentialCommandGroup
{
    public IntakeTest()
    {
        addCommands(
            Constants.intake.pivotIntake("up")
            );
        addRequirements(Constants.intake);
    }
}

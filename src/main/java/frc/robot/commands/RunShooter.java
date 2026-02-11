package frc.robot.commands;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class RunShooter extends ParallelCommandGroup
{
    public RunShooter()
    {
        addCommands(
                Constants.shooter.runShooter());

        addRequirements(getRequirements());
    }
}
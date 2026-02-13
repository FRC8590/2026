package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;

public class ShooterSetSpeed extends SequentialCommandGroup {
    
    public ShooterSetSpeed (double speed)
    {
        addCommands(
            Constants.shooter.runShooter(speed)
        );

        addRequirements(getRequirements());
    }
}

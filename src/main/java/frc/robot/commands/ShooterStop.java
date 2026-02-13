package frc.robot.commands;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ShooterStop extends SequentialCommandGroup {
    /**
     * Stops the shooter and belt
     */
    public ShooterStop() {
        addCommands(
                Constants.shooter.stopShooter(),
                Constants.belt.stopBelt());

        addRequirements(getRequirements());
    }
}

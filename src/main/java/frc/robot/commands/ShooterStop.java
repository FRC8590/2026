package frc.robot.commands;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class ShooterStop extends ParallelCommandGroup {
    /**
     * Stops the shooter and belt
     */
    public ShooterStop() {
        addCommands(
                Constants.shooter.shooterStop(),
                Constants.belt.stopBelt());

        addRequirements(getRequirements());
    }
}

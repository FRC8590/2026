package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;

public class ShooterSetSpeed extends SequentialCommandGroup {
    /**
     * Set the shooter to a specific motor speed between -1 and 1; runs the belt
     * @param speed to set the motors at.
     */
    public ShooterSetSpeed(double speed) {
        addCommands(
                Constants.shooter.runShooter(speed),
                Constants.belt.runBelt());

        addRequirements(getRequirements());
    }
}

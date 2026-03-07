package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ShooterSetSpeed;

/*
 * Primary command for shooting.
 * 
 * This aims and then runs the shooter once aligned.
 */
public class Shoot extends SequentialCommandGroup {
    public Shoot() {
        addCommands(
            //Constants.drivebase.aimAtTarget(),
            new ShooterSetSpeed(Constants.shooterspeed)
        );

    }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ShooterSetSpeed;

public class Shoot extends SequentialCommandGroup {
    public Shoot() {
        addCommands(
            Constants.drivebase.aimAtTarget(),
            new ShooterSetSpeed(Constants.shooterspeed)
        );

    }
}

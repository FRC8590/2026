package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;

public class BeltStop extends SequentialCommandGroup {
    /**
     * Stops the belt
     */
    public BeltStop() {
        addCommands(
                Constants.belt.beltStop(),
                Constants.belt.indexerStop());

        addRequirements(getRequirements());
    }
}
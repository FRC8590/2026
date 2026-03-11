package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;

public class IntakeStop extends ParallelCommandGroup {
    /**
     * Starts the intake wheels
     */
    public IntakeStop() {
        addCommands(
                Constants.intake.intakeStop());

        addRequirements(getRequirements());
    }
}

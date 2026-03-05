package frc.robot.commands;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class IntakeUp extends ParallelCommandGroup {
    /**
     * Pivots the intake up and stops the intake wheels
     */
    public IntakeUp() {
        addCommands(
                Constants.intake.intakeUp());

        addRequirements(getRequirements());
    }
}

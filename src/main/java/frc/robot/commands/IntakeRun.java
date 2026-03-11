package frc.robot.commands;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class IntakeRun extends ParallelCommandGroup {
    /**
     * Starts the intake wheels
     */
    public IntakeRun() {
        addCommands(
                Constants.intake.intakeRun());

        addRequirements(getRequirements());
    }
}

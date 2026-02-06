package frc.robot.commands;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class IntakeUp extends ParallelCommandGroup {
    public IntakeUp() {
        addCommands(
                Constants.intake.runIntake(),
                Constants.intake.pivotIntake("up"));

        addRequirements(getRequirements());
    }
}

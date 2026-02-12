package frc.robot.commands;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class IntakeDown extends SequentialCommandGroup {
    public IntakeDown() {
        addCommands(
                Constants.intake.intakeDown());

        addRequirements(getRequirements());
    }
}

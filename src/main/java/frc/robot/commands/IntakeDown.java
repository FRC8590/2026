package frc.robot.commands;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class IntakeDown extends ParallelCommandGroup {
    public IntakeDown() {
        addCommands(
                Constants.intake.stopIntake(),
                Constants.intake.pivotIntake("down"));

        addRequirements(getRequirements());
    }
}

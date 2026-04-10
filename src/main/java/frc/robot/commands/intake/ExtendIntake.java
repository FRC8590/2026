package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import lib.woodsonrobotics.SystemWrapper;

public class ExtendIntake extends Command {
    private final SystemWrapper<? extends Intake> intakeSystem;

    public ExtendIntake(SystemWrapper<? extends Intake> intake) {
        intakeSystem = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intakeSystem.ifEnabled(intake -> {
            intake.extend();
            intake.run();
        });
    }

    @Override
    public boolean isFinished() {
        var intakeOpt = intakeSystem.get();
        if (intakeOpt.isEmpty()) {
            return true;
        }

        var intake = intakeOpt.get();
        return intake.isAtSetpoint();
    }
}

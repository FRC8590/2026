package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import lib.woodsonrobotics.SystemWrapper;

public class RetractIntake extends Command {
    private final SystemWrapper<? extends Intake> intakeSystem;

    public RetractIntake(SystemWrapper<? extends Intake> intake) {
        intakeSystem = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intakeSystem.ifEnabled(intake -> {
            intake.retract();
            intake.stop();
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

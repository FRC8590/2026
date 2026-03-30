package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

import lib.woodsonrobotics.SystemWrapper;

public class RunIntake extends Command {
    private final SystemWrapper<Intake> intakeSystem;

    public RunIntake(SystemWrapper<Intake> intake) {
        intakeSystem = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intakeSystem.get().ifPresent((intake) -> {
            intake.down();
            intake.run();
        });
    }

    @Override
    public void end(boolean interrupted) {
        intakeSystem.get().ifPresent((intake) -> {
            intake.up();
            intake.stop();
        });
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

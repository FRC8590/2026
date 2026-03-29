package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class RunIntake extends Command {
    private final Intake intakeSystem;

    public RunIntake(Intake intake) {
        intakeSystem = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intakeSystem.down();
        intakeSystem.run();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSystem.up();
        intakeSystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

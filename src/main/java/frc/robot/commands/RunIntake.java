package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import lib.woodsonrobotics.SystemWrapper;

/* Bring the intake down and run the intake wheels.
 *
 * When the command is finished, the wheels will stop
 * and the intake will be brought back up.
 */
public class RunIntake extends Command {
    private final SystemWrapper<? extends Intake> intakeSystem;

    public RunIntake(SystemWrapper<? extends Intake> intake) {
        intakeSystem = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intakeSystem.ifEnabled(intake -> {
            intake.down();
            intake.run();
        });
    }

    @Override
    public void end(boolean interrupted) {
        intakeSystem.ifEnabled(intake -> {
            intake.up();
            intake.stop();
        });
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import lib.woodsonrobotics.SystemWrapper;

public class ZeroGyro extends Command {
    private final SystemWrapper<Swerve> driveSystem;

    public ZeroGyro(SystemWrapper<Swerve> drive) {
        driveSystem = drive;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        driveSystem.get().ifPresent((drive) -> drive.zeroGyro());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

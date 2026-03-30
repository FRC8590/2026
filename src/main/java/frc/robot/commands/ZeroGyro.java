package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Swerve;
import lib.woodsonrobotics.SystemWrapper;

/* Zero the gyro on the drive system. */
public class ZeroGyro extends Command {
    private final SystemWrapper<? extends Swerve> driveSystem;

    public ZeroGyro(SystemWrapper<? extends Swerve> drive) {
        driveSystem = drive;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        driveSystem.ifEnabled(drive -> drive.zeroGyro());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

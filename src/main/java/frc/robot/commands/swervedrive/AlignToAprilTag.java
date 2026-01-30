package frc.robot.commands.swervedrive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
import swervelib.SwerveInputStream;

public class AlignToAprilTag extends Command {
    private final SwerveSubsystem swerve;
    private final PIDController rotationController;
    private final SwerveInputStream inputStream;
    private double desired;

    public AlignToAprilTag(SwerveSubsystem swerve, SwerveInputStream inputStream) {
        this.swerve = swerve;
        this.inputStream = inputStream;
        
        // Tune these PID values as needed
        this.rotationController = new PIDController(4, 0, 0);
        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        
        addRequirements(swerve);
    }

    @Override
    public void execute() {

        if(Vision.seesNumber(1)){
            desired = 180;
        }
        else if(Vision.seesNumber(17)){
            desired = 90;
        }

        // Calculate the rotation speed using PID to target 0 degrees
        double rotationSpeed = rotationController.calculate(
            swerve.getHeading().getRadians(),
            Math.toRadians(desired)
        );
        
        // Get the next speeds from the input stream but override the rotation
        ChassisSpeeds speeds = inputStream.get();
        speeds.omegaRadiansPerSecond = rotationSpeed;

        
        // Drive the robot field oriented
        swerve.driveFieldOriented(speeds);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop when the command ends
        swerve.drive(new ChassisSpeeds());
    }
} 
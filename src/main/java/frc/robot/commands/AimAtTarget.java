package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.services.vision.VisionService;
import frc.robot.subsystems.drive.Swerve;
import lib.woodsonrobotics.SystemWrapper;

/**
 * Aim at the hub based on vision.
 */
public class AimAtTarget extends Command {
    private final SystemWrapper<? extends Swerve> driveSystem;
    private final VisionService visionService;
    private final PIDController rotationPID;
    private Rotation2d desiredRotation;

    public AimAtTarget(VisionService vision, SystemWrapper<? extends Swerve> drive) {
        visionService = vision;
        driveSystem = drive;
        rotationPID = new PIDController(2.0, 0.0, 0.0);
        rotationPID.setTolerance(Units.degreesToRadians(2.0));
        rotationPID.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        var driveOpt = driveSystem.get();
        if (driveOpt.isEmpty()) {
            return;
        }

        var tagPose = visionService.getTagFieldPose(RobotContainer.getHubAprilTag());
        Pose2d robotPose = driveOpt.get().getPose();
        Translation2d toTag = tagPose.getTranslation()
                .minus(robotPose.getTranslation());

        // This is the angle from the robot to the tag in field space.
        // I don't really understand this equation -- I stole it!
        desiredRotation = new Rotation2d(Math.atan2(toTag.getY(), toTag.getX()));

        rotationPID.reset();
        rotationPID.setSetpoint(desiredRotation.getRadians());
    }

    @Override
    public void execute() {
        var driveOpt = driveSystem.get();
        if (driveOpt.isEmpty() || desiredRotation == null) {
            return;
        }
        var drive = driveOpt.get();

        double currentAngle = drive.getPose().getRotation().getRadians();
        double rotationSpeed = rotationPID.calculate(currentAngle);

        drive.drive(new ChassisSpeeds(0, 0, rotationSpeed));
    }

    @Override
    public boolean isFinished() {
        return rotationPID.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        driveSystem.get().ifPresent(d -> d.drive(new ChassisSpeeds(0, 0, 0)));
    }
}
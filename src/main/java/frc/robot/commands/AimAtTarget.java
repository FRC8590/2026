package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.services.vision.VisionService;
import frc.robot.subsystems.drive.Swerve;
import lib.woodsonrobotics.SystemWrapper;

/**
 * Aim the robot at the target returned by the vision service.
 */
public class AimAtTarget extends Command {
    // Peter: I suspect we'll need this at some point
    // private final Vision visionService;
    private final SystemWrapper<? extends Swerve> driveSystem;
    private final PIDController headingController = new PIDController(5, 0, 0);

    public AimAtTarget(VisionService vision, SystemWrapper<? extends Swerve> drive) {
        // visionService = vision;
        driveSystem = drive;
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        headingController.setTolerance(Units.degreesToRadians(0.5));
        addRequirements(drive);
    }

    @Override
    public void execute() {
        /*
         * int primaryId = Vision.getHubAprilTag();
         * Optional<Pose3d> tagPoseOpt = Vision.fieldLayout.getTagPose(primaryId);
         * 
         * if (tagPoseOpt.isEmpty() || !Vision.seesNumber(primaryId)) {
         * driveSystem.drive(new ChassisSpeeds(0, 0, 0));
         * return;
         * }
         * 
         * Pose2d robotPose = driveSystem.getPose();
         * Pose2d tagPose = tagPoseOpt.get().toPose2d();
         * 
         * // Angle from robot to the tag
         * double dx = tagPose.getX() - robotPose.getX();
         * double dy = tagPose.getY() - robotPose.getY();
         * double angleToTag = Math.atan2(dy, dx);
         * 
         * double rotationSpeed = headingController.calculate(
         * robotPose.getRotation().getRadians(),
         * angleToTag);
         * 
         * driveSystem.drive(new ChassisSpeeds(0, 0, rotationSpeed));
         */
    }

    @Override
    public void end(boolean interrupted) {
        driveSystem.ifEnabled(swerve -> swerve.drive(new ChassisSpeeds(0, 0, 0)));
    }

    @Override
    public boolean isFinished() {
        return headingController.atSetpoint();
    }
}

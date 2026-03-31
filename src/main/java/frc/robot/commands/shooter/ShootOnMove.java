package frc.robot.commands.shooter;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.services.vision.VisionService;
import lib.woodsonrobotics.SystemWrapper;

/**
 * Shoot at the hub while the robot is moving.
 *
 * Uses geometric velocity compensation to adjust heading and RPM.
 * The driver retains full control of translation.
 * Rotation is overridden via a supplier wired into the teleop drive command.
 */
public class ShootOnMove extends Command {
    private final SystemWrapper<Shooter> shooterSystem;
    private final SystemWrapper<? extends Swerve> driveSystem;
    private final VisionService visionService;

    private final AtomicReference<Double> rotationOverride = new AtomicReference<>(0.0);
    private final PIDController rotationPID = new PIDController(1.5, 0.0, 0.0);

    private static final int TARGET_ID = 26;
    private static final double MIN_DISTANCE = 0.75;
    private static final double MAX_DISTANCE = 6.0;

    // Sim ball speed model: 6000 RPM = 20 m/s
    private static final double RPM_TO_SPEED = 20.0 / 6000.0;
    private static final double SPEED_TO_RPM = 6000.0 / 20.0;

    public ShootOnMove(SystemWrapper<Shooter> shooter,
            SystemWrapper<? extends Swerve> drive,
            VisionService vision) {
        shooterSystem = shooter;
        driveSystem = drive;
        visionService = vision;
        // We intentionally don't add the drive system as a requirement here,
        // because we want the drive commands to *not* be cancelled once we add
        // the rotation override.
        addRequirements(shooter);
    }

    public Supplier<Double> getRotationOverride() {
        return rotationOverride::get;
    }

    @Override
    public void initialize() {
        rotationPID.reset();
        rotationPID.enableContinuousInput(-Math.PI, Math.PI);
        rotationPID.setTolerance(Units.degreesToRadians(1.5));
        rotationOverride.set(0.0);
    }

    // Peter: This is vibecoded math shit that I don't understand.
    // It only somewhat works.
    // Ideally, we should use the ballistics simulation, but it doesn't
    // seem reliable enough at the moment to be used here.
    @Override
    public void execute() {
        var driveOpt = driveSystem.get();
        if (driveOpt.isEmpty()) {
            return;
        }
        var drive = driveOpt.get();

        var tagPoseOpt = visionService.getTagFieldPose(TARGET_ID);
        if (tagPoseOpt.isEmpty()) {
            return;
        }

        Pose2d robotPose = drive.getPose();
        Translation2d hubPos = tagPoseOpt.get().getTranslation();

        Translation2d toHub = hubPos.minus(robotPose.getTranslation());
        double distance = toHub.getNorm();

        if (distance < MIN_DISTANCE || distance > MAX_DISTANCE) {
            return;
        }

        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                drive.getRobotVelocity(), robotPose.getRotation());
        Translation2d robotVel = new Translation2d(
                fieldSpeeds.vxMetersPerSecond,
                fieldSpeeds.vyMetersPerSecond);

        Translation2d toHubUnit = toHub.div(distance);

        Translation2d perpUnit = new Translation2d(-toHubUnit.getY(), toHubUnit.getX());

        double velocityTowardHub = robotVel.getX() * toHubUnit.getX()
                + robotVel.getY() * toHubUnit.getY();
        double perpendicularVel = robotVel.getX() * perpUnit.getX()
                + robotVel.getY() * perpUnit.getY();

        double baseRPM = Shooter.distanceToRPM(distance);
        double baseBallSpeed = baseRPM * RPM_TO_SPEED;

        double adjustedBallSpeed = baseBallSpeed - velocityTowardHub;
        adjustedBallSpeed = Math.max(0, adjustedBallSpeed);
        double adjustedRPM = Math.min(adjustedBallSpeed * SPEED_TO_RPM, Shooter.SHOOTER_MAX_RPM);

        shooterSystem.ifEnabled(shooter -> shooter.setGoalRPM(adjustedRPM));

        double leadAngleRadians = Math.asin(
                Math.max(-1.0, Math.min(1.0, perpendicularVel / Math.max(baseBallSpeed, 0.1))));

        double hubAngle = Math.atan2(toHub.getY(), toHub.getX());

        Rotation2d desiredHeading = new Rotation2d(hubAngle + leadAngleRadians);

        double currentAngle = robotPose.getRotation().getRadians();
        double rotationSpeed = rotationPID.calculate(currentAngle, desiredHeading.getRadians());
        double clampedRotation = Math.max(-3.0, Math.min(3.0, rotationSpeed));
        rotationOverride.set(clampedRotation);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        rotationOverride.set(0.0);
        shooterSystem.ifEnabled(shooter -> shooter.setGoalRPM(0));
    }
}
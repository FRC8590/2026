package frc.robot.commands.shooter;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.BallisticsSim;
import frc.robot.RobotContainer;
import frc.robot.services.vision.VisionService;
import lib.woodsonrobotics.SystemWrapper;
import lib.woodsonrobotics.telemetry.notify.DriveNotifier;

/**
 * Shoot at the hub while the robot is moving.
 *
 * Uses geometric velocity compensation to adjust heading and RPM.
 * The driver retains full control of translation.
 * Rotation is overridden via a supplier wired into the teleop drive command.
 */
public class ShootWithRotationOverride extends Command {
    private final SystemWrapper<Shooter> shooterSystem;
    private final SystemWrapper<? extends Swerve> driveSystem;
    private final VisionService visionService;

    private final AtomicReference<Double> rotationOverride = new AtomicReference<>(0.0);
    private final PIDController rotationPID = new PIDController(1.5, 0.0, 0.0);

    private static final double MIN_DISTANCE = 0.75;
    private static final double MAX_DISTANCE = 6.0;

    // Sim ball speed model: 6000 RPM = 20 m/s
    private static final double RPM_TO_SPEED = 20.0 / 6000.0;
    private static final double SPEED_TO_RPM = 6000.0 / 20.0;

    private static final double SHOOTER_TO_HUB_HEIGHT = 1.83 - Units.feetToMeters(18.73 / 12);

    public ShootWithRotationOverride(SystemWrapper<Shooter> shooter,
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

    @Override
    public void execute() {
        var driveOpt = driveSystem.get();
        if (driveOpt.isEmpty()) {
            return;
        }
        var drive = driveOpt.get();

        var tagPose = visionService.getTagFieldPose(RobotContainer.getHubAprilTag());

        Pose2d robotPose = drive.getPose();
        Translation2d hubPos = tagPose.getTranslation();

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

        Translation2d firingSolution;
        try {
            firingSolution = BallisticsSim.firingSolution(
                    new Translation3d(toHub.getX(), SHOOTER_TO_HUB_HEIGHT, toHub.getY()), robotVel, 0.01, null);
        } catch (Exception e) {
            e.printStackTrace();
            DriveNotifier.internalError("ShootWithRotationOverride", "firingSolution threw exception");
            return;
        }

        shooterSystem.ifEnabled(shooter -> shooter.setGoalRPM(firingSolution.getX() * SPEED_TO_RPM));

        double currentAngle = robotPose.getRotation().getRadians();
        double rotationSpeed = rotationPID.calculate(currentAngle, Math.toRadians(firingSolution.getY()));
        double clampedRotation = Math.max(-3.0, Math.min(3.0, rotationSpeed));
        rotationOverride.set(clampedRotation);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("done");
        rotationOverride.set(0.0);
        shooterSystem.ifEnabled(shooter -> shooter.setGoalRPM(0));
    }
}
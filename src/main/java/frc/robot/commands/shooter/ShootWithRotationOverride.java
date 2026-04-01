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
import frc.robot.RobotContainer;
import frc.robot.services.vision.VisionService;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.BallisticsSim;
import lib.woodsonrobotics.SystemWrapper;
import lib.woodsonrobotics.telemetry.notify.DriveNotifier;

/**
 * Run and rotate the shooter while moving.
 *
 * Uses the ballistics simulation to compute the correct heading and RPM,
 * accounting for robot velocity and drag.
 *
 * The driver retains full control of translation.
 * Rotation is overridden via a supplier wired into the teleop drive command.
 */
public class ShootWithRotationOverride extends Command {
    private final SystemWrapper<Shooter> shooterSystem;
    private final SystemWrapper<? extends Swerve> driveSystem;
    private final VisionService visionService;

    private final AtomicReference<Double> rotationOverride = new AtomicReference<>(0.0);
    private final PIDController rotationPID = new PIDController(1.5, 0.0, 0.0);

    // Warm-start: reuse last solution angle as initial guess for the solver
    private Double lastSolverAngleRadians = null;

    private static final double MIN_DISTANCE = 0.75;
    private static final double MAX_DISTANCE = 6.0;
    private static final double ACCURACY_MARGIN = 0.05;
    private static final double SHOOTER_HEIGHT_METERS = 0.61;
    private static final double HUB_HEIGHT_METERS = 1.83;
    private static final Translation2d SHOOTER_OFFSET = new Translation2d(0.2, 0.0);

    // Sim fuel speed model: 6000 RPM = 20 m/s
    // We need to update this with the actual physics
    private static final double SPEED_TO_RPM = 6000.0 / 20.0;

    public ShootWithRotationOverride(SystemWrapper<Shooter> shooter,
            SystemWrapper<? extends Swerve> drive,
            VisionService vision) {
        shooterSystem = shooter;
        driveSystem = drive;
        visionService = vision;
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
        lastSolverAngleRadians = null;
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

        // Shooter exit point in field space
        Translation2d shooterFieldPos = robotPose.getTranslation()
                .plus(SHOOTER_OFFSET.rotateBy(robotPose.getRotation()));

        // X and Z are horizontal components, Y is vertical rise
        Translation2d hubXY = tagPose.getTranslation().minus(shooterFieldPos);
        double distance = hubXY.getNorm();

        if (distance < MIN_DISTANCE || distance > MAX_DISTANCE) {
            return;
        }

        Translation3d relativeTarget = new Translation3d(
                hubXY.getX(),
                HUB_HEIGHT_METERS - SHOOTER_HEIGHT_METERS,
                hubXY.getY());

        // This is a *field-relative* robot velocity
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                drive.getRobotVelocity(), robotPose.getRotation());
        Translation2d robotVelocity = new Translation2d(
                fieldSpeeds.vxMetersPerSecond,
                fieldSpeeds.vyMetersPerSecond);

        Translation2d solution;
        try {
            solution = BallisticsSim.firingSolution(
                    relativeTarget, robotVelocity, ACCURACY_MARGIN,
                    lastSolverAngleRadians);
        } catch (Exception e) {
            e.printStackTrace();
            DriveNotifier.internalError("ShootWithRotationOverride", "Solver threw error");
            return;
        }

        if (solution.getX() < 0) {
            // Solver timed out.
            // TODO: Peter or Isaac: We should use an Optional<> here, not a -1 sentinel.
            return;
        }

        // We cache angle for warm-starting next frame. This improves
        // performance.
        lastSolverAngleRadians = Math.toRadians(solution.getY());

        // solution.getX() = required ball speed in m/s
        // solution.getY() = absolute field heading in degrees
        double rpm = Math.min(solution.getX() * SPEED_TO_RPM, Shooter.SHOOTER_MAX_RPM);
        shooterSystem.ifEnabled(shooter -> shooter.setGoalRPM(rpm));

        Rotation2d desiredHeading = Rotation2d.fromDegrees(solution.getY());
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
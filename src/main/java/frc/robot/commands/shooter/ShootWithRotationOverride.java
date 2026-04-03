package frc.robot.commands.shooter;

import java.util.concurrent.atomic.AtomicReference;

import org.apache.commons.math3.exception.NumberIsTooSmallException;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.BallisticsSim;
import frc.robot.RobotContainer;
import frc.robot.services.RotationOverrideService;
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
    private final RotationOverrideService rotationOverrideService;

    private final PIDController rotationPID = new PIDController(3.0, 0.0, 0);

    private static final double MIN_DISTANCE = 0.75;
    private static final double MAX_DISTANCE = 6.0;

    // Sim ball speed model: 6000 RPM = 20 m/s
    private static final double RPM_TO_SPEED = 20.0 / 6000.0;
    private static final double SPEED_TO_RPM = 6000.0 / 20.0;

    private static final double SHOOTER_TO_HUB_HEIGHT = 1.83 - Units.feetToMeters(18.73 / 12);

    private Thread solverThread;

    // This is volatile to ensure sequential memory ordering.
    private volatile Double lastSolverAngleRadians = null;

    // Latest valid solution from the solver thread
    // x = speed m/s, y = heading degrees
    private final AtomicReference<Translation2d> latestSolution = new AtomicReference<>(null);

    // Inputs snapshot written by execute(), read by solver thread
    private final AtomicReference<Translation3d> latestTarget = new AtomicReference<>(null);
    private final AtomicReference<Translation2d> latestRobotVel = new AtomicReference<>(null);

    public ShootWithRotationOverride(SystemWrapper<Shooter> shooter,
            SystemWrapper<? extends Swerve> drive,
            VisionService vision, RotationOverrideService rotationOverride) {
        shooterSystem = shooter;
        driveSystem = drive;
        visionService = vision;
        rotationOverrideService = rotationOverride;
        // We intentionally don't add the drive system as a requirement here,
        // because we want the drive commands to *not* be cancelled once we add
        // the rotation override.
        addRequirements(shooter);
    }

    public boolean isHeadingAligned() {
        return rotationPID.atSetpoint();
    }

    @Override
    public void initialize() {
        rotationPID.reset();
        rotationPID.enableContinuousInput(-Math.PI, Math.PI);
        rotationPID.setTolerance(Units.degreesToRadians(1.5));
        rotationOverrideService.clearOverride();
        solverThread = new Thread(() -> {
            Translation3d lastSolvedTarget = null;
            Translation2d lastSolvedVel = null;

            while (!Thread.currentThread().isInterrupted()) {
                Translation3d target = latestTarget.get();
                Translation2d vel = latestRobotVel.get();

                if (target == null || vel == null) {
                    System.out.println("Solver: waiting for inputs, target=" + target + " vel=" + vel);
                } else {
                    System.out.println("Solver: running with target=" + target + " vel=" + vel);
                }

                if (target != null && vel != null) {
                    // Only solve if inputs have changed meaningfully
                    boolean shouldSolve = lastSolvedTarget == null
                            || target.minus(lastSolvedTarget).getNorm() > 0.05
                            || vel.minus(lastSolvedVel).getNorm() > 0.1;

                    if (shouldSolve) {
                        Double warmStart = lastSolverAngleRadians;

                        // If target has changed significantly, cold-start to force full re-solve
                        if (lastSolvedTarget != null &&
                                target.minus(lastSolvedTarget).getNorm() > 0.1) {
                            warmStart = null; // force cold start
                        }

                        // Skip the solve if robot speed is too high for the ballistics to handle
                        if (vel.getNorm() > 2.5) {
                            // Fall back to simple geometric aim to just point at the hub
                            double hubAngle = Math.atan2(target.getZ(), target.getX());
                            double speed = /* use distanceToRPM logic */ Shooter.distanceToRPM(
                                    Math.hypot(target.getX(), target.getZ())) * 20.0 / 6000.0;
                            latestSolution.set(new Translation2d(speed, Math.toDegrees(hubAngle)));
                            lastSolvedTarget = target;
                            lastSolvedVel = vel;
                            continue;
                        }

                        Translation2d solution;
                        try {
                            solution = BallisticsSim.firingSolution(
                                    target, vel, 0.01, warmStart);
                        } catch (NumberIsTooSmallException e) {
                            // TODO: Peter: The ballistics simulation should fail cleanly
                            // if the target is impossible.
                            shooterSystem.ifEnabled(shooter -> shooter.setGoalRPM(0));
                            continue;
                        } catch (Exception e) {
                            e.printStackTrace();
                            DriveNotifier.internalError("ShootWithRotationOverride",
                                    "firingSolution threw: " + e.getMessage());
                            continue;
                        }

                        if (solution.getX() >= 0) {
                            lastSolverAngleRadians = Math.toRadians(solution.getY());
                            lastSolvedTarget = target;
                            lastSolvedVel = vel;
                            latestSolution.set(solution);
                        }
                    }
                }

                try {
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                    return;
                }
            }
        }, "SOTM-Solver");
        solverThread.start();

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
        latestTarget.set(new Translation3d(toHub.getX(), SHOOTER_TO_HUB_HEIGHT, toHub.getY()));
        latestRobotVel.set(new Translation2d(
                fieldSpeeds.vxMetersPerSecond,
                fieldSpeeds.vyMetersPerSecond));

        Translation2d solution = latestSolution.get();
        if (solution == null) {
            // No solution yet
            rotationOverrideService.setOverride(0.0);
            return;
        }

        double rpm = Math.min(solution.getX() * SPEED_TO_RPM, Shooter.SHOOTER_MAX_RPM);
        shooterSystem.ifEnabled(shooter -> shooter.setGoalRPM(rpm));

        double currentAngle = robotPose.getRotation().getRadians();
        double maxOmega = driveOpt.get().getSwerveDrive().getMaximumChassisAngularVelocity();
        double rotationSpeed = rotationPID.calculate(currentAngle, Math.toRadians(solution.getY()));
        rotationOverrideService.setOverride(rotationSpeed / maxOmega);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        rotationOverrideService.clearOverride();
        if (solverThread != null) {
            solverThread.interrupt();
        }
        shooterSystem.ifEnabled(shooter -> shooter.setGoalRPM(0));
    }
}
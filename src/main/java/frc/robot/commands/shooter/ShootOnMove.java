package frc.robot.commands.shooter;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.services.vision.VisionService;
import frc.robot.utils.BallisticsSim;
import lib.woodsonrobotics.SystemWrapper;

/**
 * Shoot at the hub while the robot is moving.
 *
 * Uses a full ballistics simulation with drag to compute the correct
 * heading and RPM, accounting for robot velocity.
 *
 * The driver retains full control of translation. Rotation is overridden
 * via a supplier wired into the teleop drive command.
 */
public class ShootOnMove extends Command {
    private final SystemWrapper<Shooter> shooterSystem;
    private final SystemWrapper<? extends Swerve> driveSystem;
    private final VisionService visionService;

    private final AtomicReference<Double> rotationOverride = new AtomicReference<>(0.0);
    private final AtomicReference<Translation2d> latestSolution = new AtomicReference<>(null);
    private final AtomicReference<Translation2d> latestHubXY = new AtomicReference<>(null);
    private final AtomicReference<Double> lastHeadingRadians = new AtomicReference<>(null);
    private final AtomicReference<Boolean> solving = new AtomicReference<>(false);

    private final PIDController rotationPID = new PIDController(1.5, 0.0, 0.05);
    private Thread solverThread;

    private static final Translation2d SHOOTER_OFFSET = new Translation2d(0.2, 0.0);
    private static final double SHOOTER_HEIGHT_METERS = 0.61;
    private static final double HUB_HEIGHT_METERS = 1.83;
    private static final int TARGET_ID = 26;
    private static final double ACCURACY_MARGIN = 0.05;

    public ShootOnMove(SystemWrapper<Shooter> shooter,
            SystemWrapper<? extends Swerve> drive,
            VisionService vision) {
        shooterSystem = shooter;
        driveSystem = drive;
        visionService = vision;
        shooterSystem.get().ifPresent(this::addRequirements);
    }

    public Supplier<Double> getRotationOverride() {
        return rotationOverride::get;
    }

    @Override
    public void initialize() {
        lastRobotPos = null;
        lastRobotVel = null;
        // Interrupt and abandon old thread. Waiting on it can result in
        // long startup times for the command.
        if (solverThread != null) {
            solverThread.interrupt();
        }
        solving.set(false);
        rotationOverride.set(0.0);
        latestSolution.set(null);
        latestHubXY.set(null);
        lastHeadingRadians.set(null);

        solverThread = new Thread(() -> {
            while (!Thread.currentThread().isInterrupted()) {
                computeAndCacheSolution();
                try {
                    Thread.sleep(200);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        });
        solverThread.setDaemon(true);
        solverThread.start();
    }

    private Translation2d lastRobotPos = null;
    private Translation2d lastRobotVel = null;

    private void computeAndCacheSolution() {
        if (solving.get())
            return;
        if (Thread.currentThread().isInterrupted())
            return;
        solving.set(true);

        try {
            var driveOpt = driveSystem.get();
            if (driveOpt.isEmpty())
                return;
            var drive = driveOpt.get();

            var tagPoseOpt = visionService.getTagFieldPose(TARGET_ID);
            if (tagPoseOpt.isEmpty()) {
                // We should probably throw an exception here?
                return;
            }

            Pose2d robotPose = drive.getPose();
            Translation2d shooterFieldPos = robotPose.getTranslation()
                    .plus(SHOOTER_OFFSET.rotateBy(robotPose.getRotation()));

            ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                    drive.getRobotVelocity(), robotPose.getRotation());
            Translation2d robotVelocity = new Translation2d(
                    fieldSpeeds.vxMetersPerSecond,
                    fieldSpeeds.vyMetersPerSecond);

            Translation2d currentPos = robotPose.getTranslation();
            if (!hasChangedEnough(currentPos, robotVelocity))
                return;
            lastRobotPos = currentPos;
            lastRobotVel = robotVelocity;

            Translation2d hubXY = tagPoseOpt.get().getTranslation().minus(shooterFieldPos);
            double distance = hubXY.getNorm();

            if (distance < 0.75 || distance > 6.0) {
                System.err.println("ShootOnMove: distance " + distance + "m outside viable range");
                return;
            }

            double flightTime = distance / (10.0 * Math.cos(Math.toRadians(69)));

            Translation2d projectedRobotPos = robotPose.getTranslation().plus(
                    new Translation2d(
                            fieldSpeeds.vxMetersPerSecond * flightTime,
                            fieldSpeeds.vyMetersPerSecond * flightTime));
            Translation2d projectedShooterPos = projectedRobotPos
                    .plus(SHOOTER_OFFSET.rotateBy(robotPose.getRotation()));
            hubXY = tagPoseOpt.get().getTranslation().minus(projectedShooterPos);

            distance = hubXY.getNorm();
            if (distance < 0.75 || distance > 6.0) {
                System.err.println("ShootOnMove: projected distance " + distance + "m outside viable range");
                return;
            }

            Translation3d relativeTarget = new Translation3d(
                    hubXY.getX(),
                    HUB_HEIGHT_METERS - SHOOTER_HEIGHT_METERS,
                    hubXY.getY());

            if (Thread.currentThread().isInterrupted())
                return;

            Translation2d solution;
            try {
                solution = BallisticsSim.firingSolution(
                        relativeTarget, robotVelocity, ACCURACY_MARGIN,
                        lastHeadingRadians.get());
            } catch (Exception e) {
                System.err.println("ShootOnMove: solver threw exception: " + e.getMessage());
                return;
            }

            if (solution.getX() < 0) {
                System.err.println("ShootOnMove: firingSolution failed");
                return;
            }

            if (Thread.currentThread().isInterrupted()) {
                return;
            }

            lastHeadingRadians.set(Math.toRadians(solution.getY()));
            latestSolution.set(solution);
            latestHubXY.set(hubXY);
        } finally {
            if (!Thread.currentThread().isInterrupted()) {
                solving.set(false);
            }
        }
    }

    private boolean hasChangedEnough(Translation2d currentPos, Translation2d currentVel) {
        if (lastRobotPos == null || lastRobotVel == null) {
            return true;
        }
        double posDelta = currentPos.getDistance(lastRobotPos);
        double velDelta = currentVel.minus(lastRobotVel).getNorm();
        return posDelta > 0.05 || velDelta > 0.1;
    }

    @Override
    public void execute() {
        Translation2d solution = latestSolution.get();
        Translation2d hubXY = latestHubXY.get();
        if (solution == null || hubXY == null) {
            return;
        }

        var driveOpt = driveSystem.get();
        if (driveOpt.isEmpty()) {
            return;
        }
        var drive = driveOpt.get();

        double rpm = Math.min(solution.getX() * 6000.0 / 20.0, Shooter.SHOOTER_MAX_RPM);
        shooterSystem.ifEnabled(shooter -> shooter.setGoalRPM(rpm));

        Rotation2d desiredHeading = Rotation2d.fromDegrees(solution.getY());

        double currentAngle = drive.getPose().getRotation().getRadians();
        double rotationSpeed = rotationPID.calculate(currentAngle, desiredHeading.getRadians());
        double clampedRotation = Math.max(-3.0, Math.min(3.0, rotationSpeed)); // rad/s
        rotationOverride.set(clampedRotation);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        if (solverThread != null) {
            solverThread.interrupt();
        }
        rotationOverride.set(0.0);
        shooterSystem.ifEnabled(shooter -> shooter.setGoalRPM(0));
    }
}
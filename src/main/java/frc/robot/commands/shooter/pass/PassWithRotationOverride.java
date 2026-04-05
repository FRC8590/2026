package frc.robot.commands.shooter.pass;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.RobotContainer;
import frc.robot.services.RotationOverrideService;
import frc.robot.services.vision.VisionService;
import lib.woodsonrobotics.SystemWrapper;

/**
 * Lob fuel from the neutral zone or opposing side to a landing zone in our
 * alliance zone, so a teammate (or us later) can collect and score.
 */
public class PassWithRotationOverride extends Command {
    private final SystemWrapper<Shooter> shooterSystem;
    private final SystemWrapper<? extends Swerve> driveSystem;
    private final VisionService visionService;
    public final RotationOverrideService rotationOverrideService;

    private final PIDController rotationPID = new PIDController(5.0, 0.0, 0.15);

    private static final double SPEED_TO_RPM = 6000.0 / 20.0;
    private static final double LAUNCH_ANGLE_RAD = Math.toRadians(69.0);
    private static final double G = 9.81;

    private double maxAngularVelocity = 6.28;

    // Outpost AprilTag IDs
    private static final int[] BLUE_OUTPOST_TAGS = { 29, 30 };
    private static final int[] RED_OUTPOST_TAGS = { 13, 14 };

    // How far from the outpost (toward field center) to place the landing zone
    // (meters)
    private static final double LANDING_OFFSET_X = 3.5;
    // How far from the output (toward alliance wall) to place the landing zone
    // (meters)
    private static final double LANDING_OFFSET_Y = 3.5;

    private static final double MIN_PASS_DISTANCE = 3.0;
    private static final double MAX_PASS_DISTANCE = 16.0;
    private static final double MAX_LEAD_RAD = Math.toRadians(80);

    public PassWithRotationOverride(
            SystemWrapper<Shooter> shooter,
            SystemWrapper<? extends Swerve> drive,
            VisionService vision,
            RotationOverrideService rotationOverride) {
        shooterSystem = shooter;
        driveSystem = drive;
        visionService = vision;
        rotationOverrideService = rotationOverride;
        addRequirements(shooter);
    }

    public boolean isHeadingAligned() {
        return rotationPID.atSetpoint();
    }

    @Override
    public void initialize() {
        rotationPID.reset();
        rotationPID.enableContinuousInput(-Math.PI, Math.PI);
        rotationPID.setTolerance(Units.degreesToRadians(3.0));
        rotationOverrideService.clearOverride();

        driveSystem.get().ifPresent(swerve -> {
            maxAngularVelocity = swerve.getSwerveDrive().getMaximumChassisAngularVelocity();
            if (maxAngularVelocity <= 0) {
                maxAngularVelocity = 6.28;
            }
        });
    }

    @Override
    public void execute() {
        var driveOpt = driveSystem.get();
        if (driveOpt.isEmpty()) {
            return;
        }
        var drive = driveOpt.get();

        Pose2d robotPose = drive.getPose();
        Translation2d landingZone = getLandingZone(robotPose);
        Translation2d toTarget = landingZone.minus(robotPose.getTranslation());
        double distance = toTarget.getNorm();

        if (distance < MIN_PASS_DISTANCE || distance > MAX_PASS_DISTANCE) {
            rotationOverrideService.setOverride(0.0);
            return;
        }

        // Peter: Most of the math here was assisted by Claude
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                drive.getRobotVelocity(), robotPose.getRotation());
        double vx = fieldSpeeds.vxMetersPerSecond;
        double vy = fieldSpeeds.vyMetersPerSecond;

        double bearingRad = Math.atan2(toTarget.getY(), toTarget.getX());
        double cosBearing = Math.cos(bearingRad);
        double sinBearing = Math.sin(bearingRad);
        double vToward = vx * cosBearing + vy * sinBearing;
        double vLateral = -vx * sinBearing + vy * cosBearing;

        // Ground-to-ground range with inherited horizontal velocity:
        // distance = 2*v*sin(θ) * (v*cos(θ) + vToward) / g
        //
        // Rearranged: sin(2θ)*v² + 2*sin(θ)*vToward*v - distance*g = 0
        double sinTheta = Math.sin(LAUNCH_ANGLE_RAD);
        double sin2theta = Math.sin(2 * LAUNCH_ANGLE_RAD);

        double a = sin2theta;
        double b = 2 * sinTheta * vToward;
        double c = -distance * G;
        double discriminant = b * b - 4 * a * c;

        if (discriminant < 0) {
            rotationOverrideService.setOverride(0.0);
            return;
        }

        double launchSpeed = (-b + Math.sqrt(discriminant)) / (2 * a);
        if (launchSpeed <= 0) {
            rotationOverrideService.setOverride(0.0);
            return;
        }

        double rpm = Math.min(launchSpeed * SPEED_TO_RPM, Shooter.SHOOTER_MAX_RPM);
        final double rpmFinal = rpm;
        shooterSystem.ifEnabled(shooter -> shooter.setGoalRPM(rpmFinal));

        double flightTime = 2 * launchSpeed * sinTheta / G;
        double lateralDrift = vLateral * flightTime;
        double leadAngle = -Math.atan2(lateralDrift, distance);
        leadAngle = Math.max(-MAX_LEAD_RAD, Math.min(MAX_LEAD_RAD, leadAngle));

        double desiredHeadingRad = bearingRad + leadAngle;

        double currentHeadingRad = robotPose.getRotation().getRadians();
        double pidOutput = rotationPID.calculate(currentHeadingRad, desiredHeadingRad);
        double normalized = Math.max(-1.0, Math.min(1.0, pidOutput / maxAngularVelocity));
        rotationOverrideService.setOverride(normalized);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        rotationOverrideService.clearOverride();
        shooterSystem.ifEnabled(shooter -> shooter.setGoalRPM(0));
    }

    /**
     * Derive the landing zone from the nearest outpost AprilTag.
     * Offsets toward field center to land in open carpet between wall and hub.
     */
    private Translation2d getLandingZone(Pose2d robotPose) {
        int[] tags = RobotContainer.isRedAlliance() ? RED_OUTPOST_TAGS : BLUE_OUTPOST_TAGS;
        var nearest = visionService.findNearestTag(tags, robotPose);
        Translation2d outpostPos = nearest.tagPose().getTranslation();

        // Offset toward field center: +X for blue (wall at X≈0), -X for red (wall at
        // X~=16.5)
        double dir = RobotContainer.isRedAlliance() ? -1.0 : 1.0;
        return new Translation2d(
                outpostPos.getX() + dir * LANDING_OFFSET_X,
                outpostPos.getY() + dir * LANDING_OFFSET_Y);
    }
}

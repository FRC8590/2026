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
import frc.robot.commands.DriveUnderTrench;
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

    private static final double LAUNCH_ANGLE_RAD = Math.toRadians(69.0);
    private static final double G = 9.81;

    private double maxAngularVelocity = 6.28;

    // Outpost AprilTag IDs
    private static final int[] BLUE_OUTPOST_TAGS = { 29, 30 };
    private static final int[] RED_OUTPOST_TAGS = { 13, 14 };

    // How far from the outpost (toward field center) to place the landing zone
    // (meters)
    private static final double LANDING_OFFSET_X = 3.0;

    // How far from the output (toward alliance wall) to place the landing zone
    // (meters)
    // When we're on the left side of the field
    private static final double LANDING_OFFSET_Y_LEFT = 1.0;
    // When we're on the right side of the field (but still relative to the outpost)
    private static final double LANDING_OFFSET_Y_RIGHT = 5.0;

    private static final double MIN_PASS_DISTANCE = 3.0;
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

    // Peter: Claude made this, not sure what it is, but I trust the math.
    // Flight time coefficient for a ground-to-ground lob at 69 degrees.
    // Derived from range equation: t = 2*sin(θ) * sqrt(d / (g*sin(2θ)))
    // which simplifies to t ~= 0.729 * sqrt(distance) for θ=69 degrees.
    private static final double FLIGHT_TIME_COEFFICIENT = 0.729;

    // Peter: Claude assisted with the math here
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

        if (distance < MIN_PASS_DISTANCE) {
            rotationOverrideService.setOverride(0.0);
            return;
        }

        // Robot velocity in field frame
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                drive.getRobotVelocity(), robotPose.getRotation());
        double vx = fieldSpeeds.vxMetersPerSecond;
        double vy = fieldSpeeds.vyMetersPerSecond;

        // Decompose velocity relative to target direction
        double bearingRad = Math.atan2(toTarget.getY(), toTarget.getX());
        double cosBearing = Math.cos(bearingRad);
        double sinBearing = Math.sin(bearingRad);
        double vToward = vx * cosBearing + vy * sinBearing;
        double vLateral = -vx * sinBearing + vy * cosBearing;

        // The ball inherits vToward from the robot, so it travels farther
        // than intended. Reduce the effective distance to compensate.
        // Flight time estimate: t ~= 0.729 * sqrt(distance) (from range equation at 69
        // degrees)
        double flightTime = FLIGHT_TIME_COEFFICIENT * Math.sqrt(distance);
        double effectiveDistance = distance - vToward * flightTime;

        if (effectiveDistance < 0.5) {
            // Moving so fast toward target that no shot is needed
            rotationOverrideService.setOverride(0.0);
            return;
        }

        double rpm = Shooter.distanceToRPM(effectiveDistance);
        if (rpm <= 0) {
            rotationOverrideService.setOverride(0.0);
            return;
        }
        final double rpmFinal = rpm;
        shooterSystem.ifEnabled(shooter -> shooter.setGoalRPM(rpmFinal));

        // Heading: compensate for lateral drift
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

        int[] trenchTags = RobotContainer.isRedAlliance()
                ? DriveUnderTrench.RED_EXIT_TAG_IDS
                : DriveUnderTrench.BLUE_EXIT_TAG_IDS;
        var nearestTrench = visionService.findNearestTag(trenchTags, robotPose);
        double landingOffsetY = nearestTrench.tagId() == trenchTags[0]
                ? LANDING_OFFSET_Y_LEFT
                : LANDING_OFFSET_Y_RIGHT;

        // Offset toward field center: +X for blue (wall at X≈0), -X for red (wall at
        // X~=16.5)
        double dir = RobotContainer.isRedAlliance() ? -1.0 : 1.0;
        return new Translation2d(
                outpostPos.getX() + dir * LANDING_OFFSET_X,
                outpostPos.getY() + dir * landingOffsetY);
    }
}

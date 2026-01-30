package frc.robot.commands.swervedrive;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/**
 * PreciseAlignment command that drives the robot to a target pose with high precision.
 * Uses a two-phase approach with rough alignment followed by precise alignment.
 */
public class PreciseAlignment extends Command {
    private final SwerveSubsystem m_driveSubsystem;
    private final Pose2d m_targetPose;
    
    // Controllers for different phases of alignment
    private final HolonomicDriveController m_roughController;
    private final HolonomicDriveController m_preciseController;
    
    // Telemetry publishers
    private final DoublePublisher m_distancePublisher;
    private final DoublePublisher m_angleErrorPublisher;
    
    // State tracking
    private boolean m_roughAlignmentComplete = false;
    private boolean m_commandDone = false;
    
    // Timing
    private final Timer m_commandTimer = new Timer();
    private final double m_timeoutSeconds;
    private static final double DEFAULT_TIMEOUT = 3.0;

    /**
     * Creates a new PreciseAlignment command.
     * 
     * @param driveSubsystem The swerve drive subsystem
     * @param targetPose The target pose to align to
     */
    public PreciseAlignment(SwerveSubsystem driveSubsystem, Pose2d targetPose) {
        this(driveSubsystem, targetPose, DEFAULT_TIMEOUT);
    }

    /**
     * Creates a new PreciseAlignment command with custom timeout.
     * 
     * @param driveSubsystem The swerve drive subsystem
     * @param targetPose The target pose to align to
     * @param timeoutSeconds Command timeout in seconds
     */
    public PreciseAlignment(SwerveSubsystem driveSubsystem, Pose2d targetPose, double timeoutSeconds) {
        m_driveSubsystem = driveSubsystem;
        m_targetPose = targetPose;
        m_timeoutSeconds = timeoutSeconds;
        
        // Configure rough alignment controller (wider tolerances)
        m_roughController = new HolonomicDriveController(
            new PIDController(3, 0, 0),
            new PIDController(3, 0, 0),
            new ProfiledPIDController(4, 0, 0, 
                new TrapezoidProfile.Constraints(3, 6))
        );
        m_roughController.setTolerance(new Pose2d(0.15, 0.15, Rotation2d.fromDegrees(1)));

        // Configure precise alignment controller (tighter tolerances)
        m_preciseController = new HolonomicDriveController(
            new PIDController(13, 0, 0),
            new PIDController(13, 0, 0),
            new ProfiledPIDController(10, 0, 0, 
                new TrapezoidProfile.Constraints(3, 6))
        );
        m_preciseController.setTolerance(new Pose2d(0.02, 0.02, Rotation2d.fromDegrees(1)));
        
        // Setup telemetry publishers
        m_distancePublisher = NetworkTableInstance.getDefault()
            .getTable("AutoAlignment")
            .getDoubleTopic("DistanceToTarget")
            .publish();
            
        m_angleErrorPublisher = NetworkTableInstance.getDefault()
            .getTable("AutoAlignment")
            .getDoubleTopic("AngleError")
            .publish();
            
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("PreciseAlignment: initializing...");
        m_commandTimer.reset();
        m_commandTimer.start();
        m_roughAlignmentComplete = false;
        m_commandDone = false;
        SmartDashboard.putString("Alignment Phase", "Rough Alignment");
    }

    @Override
    public void execute() {
        // Get current robot pose
        Pose2d currentPose = m_driveSubsystem.getPose();
        
        // Calculate metrics for telemetry
        double distance = currentPose.getTranslation().getDistance(m_targetPose.getTranslation());
        double angleError = Math.abs(currentPose.getRotation().minus(m_targetPose.getRotation()).getDegrees());
        
        // Update telemetry
        m_distancePublisher.set(distance);
        m_angleErrorPublisher.set(angleError);
        SmartDashboard.putNumber("Distance to Target", distance);
        SmartDashboard.putNumber("Angle Error", angleError);
        
        // Check for timeout
        if (m_commandTimer.hasElapsed(m_timeoutSeconds)) {
            System.out.println("PreciseAlignment: timeout reached");
            m_commandDone = true;
            return;
        }

        // Determine which phase we're in and calculate appropriate control
        ChassisSpeeds controlOutput;
        if (!m_roughAlignmentComplete) {
            // Rough alignment phase
            controlOutput = m_roughController.calculate(
                currentPose, m_targetPose, 0, m_targetPose.getRotation()
            );
            
            // Check if rough alignment is complete
            if (m_roughController.atReference()) {
                m_roughAlignmentComplete = true;
                System.out.println("PreciseAlignment: rough alignment complete, starting precise alignment");
                SmartDashboard.putString("Alignment Phase", "Precise Alignment");
            }
        } else {
            // Precise alignment phase
            controlOutput = m_preciseController.calculate(
                currentPose, m_targetPose, 0, m_targetPose.getRotation()
            );
            
            // Check if precise alignment is complete
            if (m_preciseController.atReference()) {
                System.out.println("PreciseAlignment: precise alignment complete");
                m_commandDone = true;
            }
        }
        
        // Apply calculated chassis speeds to robot
        m_driveSubsystem.setChassisSpeeds(controlOutput);

        // Log PID details for debugging
        SmartDashboard.putNumber("PID Error X", 
            m_roughAlignmentComplete ? 
            currentPose.getX() - m_targetPose.getX() : 
            currentPose.getX() - m_targetPose.getX());
            
        SmartDashboard.putNumber("PID Error Y", 
            m_roughAlignmentComplete ? 
            currentPose.getY() - m_targetPose.getY() : 
            currentPose.getY() - m_targetPose.getY());
            
        SmartDashboard.putNumber("PID Error Rotation", 
            m_roughAlignmentComplete ? 
            currentPose.getRotation().minus(m_targetPose.getRotation()).getDegrees() : 
            currentPose.getRotation().minus(m_targetPose.getRotation()).getDegrees());
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot
        m_driveSubsystem.setChassisSpeeds(new ChassisSpeeds());
        m_commandTimer.stop();
        
        if (interrupted) {
            System.out.println("PreciseAlignment: command interrupted");
        } else {
            System.out.println("PreciseAlignment: command completed successfully");
        }
        
        SmartDashboard.putString("Alignment Phase", interrupted ? "Interrupted" : "Completed");
    }

    @Override
    public boolean isFinished() {
        return m_commandDone;
    }
    
    /**
     * Factory method to create a PreciseAlignment command for the closest AprilTag.
     * 
     * @param swerveSubsystem The swerve subsystem
     * @return A command that will align to the closest AprilTag
     */
    public static Command alignToClosestAprilTag(SwerveSubsystem swerveSubsystem) {
        return new Command() {
            private Pose2d targetPose;
            private PreciseAlignment alignmentCommand;
            private boolean commandDone = false;
            
            @Override
            public void initialize() {
                System.out.println("Aligning to closest AprilTag...");
                int tagId = swerveSubsystem.findClosestAprilTag();
                System.out.println("Selected AprilTag ID: " + tagId);
                
                // Get the tag pose
                targetPose = swerveSubsystem.aprilTagFieldLayout.getTagPose(tagId).get().toPose2d();
                System.out.println("Target pose: " + targetPose.toString());
                
                // Create the alignment command
                alignmentCommand = new PreciseAlignment(swerveSubsystem, targetPose);
                alignmentCommand.initialize();
                commandDone = false;
            }
            
            @Override
            public void execute() {
                if (alignmentCommand != null) {
                    alignmentCommand.execute();
                    
                    // Check if the alignment command is finished
                    if (alignmentCommand.isFinished()) {
                        commandDone = true;
                    }
                }
            }
            
            @Override
            public void end(boolean interrupted) {
                if (alignmentCommand != null) {
                    alignmentCommand.end(interrupted);
                }
                System.out.println("AprilTag alignment " + (interrupted ? "interrupted" : "completed"));
            }
            
            @Override
            public boolean isFinished() {
                return commandDone;
            }
            
            @Override
            public String getName() {
                return "AlignToClosestAprilTag";
            }
        };
    }
} 
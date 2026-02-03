// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import au.grapplerobotics.LaserCan;

import java.io.File;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.math.Matter;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.OperatorConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.BeltConstants;
import frc.robot.constants.ClimbConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
    private Constants() {} // Prevent instantiation

    // Vision & Field Constants
    
    //private static final List<AprilTag> tagList = new ArrayList<AprilTag>() {{
      //  add(tag1);
  //  }};

    // public static final AprilTagFieldLayout layout = new AprilTagFieldLayout(
    //     tagList, 
    //     7.62,    // Field length (meters)
    //     3.6068   // Field width (meters)
    // );

    public static final AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    // Robot Physical Properties
    public static final double ROBOT_MASS = Units.lbsToKilograms(135); // TO CHANGE
    public static final Matter CHASSIS = new Matter(
        new Translation3d(0, 0, Units.inchesToMeters(14)), 
        ROBOT_MASS
    );
    
    // Control Loop Timing
    public static final double LOOP_TIME = 0.05; // seconds
    public static final double MAX_SPEED = 6.0;  // meters per second
    public static double visionTimerOffset = 0;

    // Subsystem Instances
    public static final SwerveSubsystem drivebase = new SwerveSubsystem(
        new File(Filesystem.getDeployDirectory(), "swerve/neo")
    );
    public static Vision vision;

    public static double scaleFactor = 1;

    public static int[] SCORING_IDS = {};

    // Constants Records
    public static final OperatorConstants OPERATOR_CONSTANTS = OperatorConstants.DEFAULT;
    public static final IntakeConstants INTAKE_CONSTANTS = IntakeConstants.DEFAULT;
    public static final BeltConstants BELT_CONSTANTS = BeltConstants.DEFAULT;
    public static final ShooterConstants SHOOTER_CONSTANTS = ShooterConstants.DEFAULT;
    public static final ClimbConstants CLIMB_CONSTANTS = ClimbConstants.DEFAULT;

    public static final LaserCan laserCan = new LaserCan(8);
    public static int lockTimer = 0;

    // Enums
    public enum ScoreLocation { // TO CHANGE
        LEFT2, 
        LEFT3, 
        RIGHT2, 
        RIGHT3
    }

    public static DriveConstants DRIVE_CONSTANTS = new DriveConstants(lockTimer, MAX_SPEED, LOOP_TIME, null, null);
}
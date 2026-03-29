// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
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
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    private Constants() {
    } // Prevent instantiation
      // Subsystem Instances

    // Constants Records
    public static final BeltConstants BELT_CONSTANTS = BeltConstants.DEFAULT;
    public static final DriveConstants DRIVE_CONSTANTS = DriveConstants.DEFAULT;
    public static final ClimbConstants CLIMB_CONSTANTS = ClimbConstants.DEFAULT;
    public static final IntakeConstants INTAKE_CONSTANTS = IntakeConstants.DEFAULT;
    public static final ShooterConstants SHOOTER_CONSTANTS = ShooterConstants.DEFAULT;
    public static final OperatorConstants OPERATOR_CONSTANTS = OperatorConstants.DEFAULT;

    // Subsystem Instances

    public static final Swerve drivebase = new Swerve(
            new File(Filesystem.getDeployDirectory(), "swerve/neo"));
    public static Vision vision = new Vision(() -> drivebase.getPose());
    public static Belt belt = new Belt();
    public static Intake intake = new Intake();
    public static Shooter shooter = new Shooter();
}
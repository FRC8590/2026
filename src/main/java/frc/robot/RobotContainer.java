// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.commands.*;
import frc.robot.utils.*;
import frc.robot.Constants;

import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public final CommandXboxController driverXbox = new CommandXboxController(0);
  public final CommandXboxController operatorController = new CommandXboxController(1);

  // auto list object
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // The robot's subsystems and commands are defined here...
  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the rotational velocity
  // buttons are quick rotation positions to different ways to face
  // WARNING: default buttons are on the same buttons as the ones defined in
  // configureBindings
  AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(Constants.drivebase,
      () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
          Constants.OPERATOR_CONSTANTS.leftYDeadband()),
      () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
          Constants.OPERATOR_CONSTANTS.leftYDeadband()),
      () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
          Constants.OPERATOR_CONSTANTS.leftYDeadband()),
      driverXbox.getHID()::getYButtonPressed,
      driverXbox.getHID()::getAButtonPressed,
      driverXbox.getHID()::getXButtonPressed,
      driverXbox.getHID()::getBButtonPressed);

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(Constants.drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * getSide() * Constants.scaleFactor,
      () -> driverXbox.getLeftX() * getSide() * Constants.scaleFactor)
      .withControllerRotationAxis(() -> -driverXbox.getRightX() * 0.72 * Constants.scaleFactor)
      .deadband(Constants.OPERATOR_CONSTANTS.deadband())
      .robotRelative(false)
      .allianceRelativeControl(false);

  SwerveInputStream driveRobotOriented = SwerveInputStream.of(Constants.drivebase.getSwerveDrive(),
      () -> -driverXbox.getLeftY() * Constants.scaleFactor,
      () -> -driverXbox.getLeftX() * Constants.scaleFactor)
      .withControllerRotationAxis(() -> -driverXbox.getRightX() * 0.6 * Constants.scaleFactor)
      .deadband(Constants.OPERATOR_CONSTANTS.deadband())
      .robotRelative(true)
      .scaleTranslation(Constants.scaleFactor);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative
   * input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
      .withControllerHeadingAxis(() -> -driverXbox.getRightX(),
          () -> -driverXbox.getRightY())
      .headingWhile(true);

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the desired angle NOT angular rotation
  Command driveFieldOrientedDirectAngle = Constants.drivebase.driveFieldOriented(driveDirectAngle);
  Command driveRobotOrientedAngular = Constants.drivebase.driveRobotRelative(driveRobotOriented);

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot
  Command driveFieldOrientedAnglularVelocity = Constants.drivebase.driveFieldOriented(driveAngularVelocity);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    // set up the shuffleboard tab
    Shuffleboard.getTab("Autonomous")
        .add("Auto Selector", m_chooser)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .withPosition(0, 0)
        .withSize(2, 1);

    Shuffleboard.getTab("Autonomous")
        .add("On Red Side?", true)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .getEntry();

    //new IntakeDown().schedule();

    // Initialize with proper alliance orientation
    Constants.drivebase.zeroGyroWithAlliance();
  }

  public int getSide() {

    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
      return 1;
    } else {
      return -1;
    }

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary predicate, or via the
   * named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
   * Flight joysticks}.
   */
  private void configureBindings() {
    assert (!RobotBase.isSimulation());
    Constants.drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    
    Constants.shooter.setDefaultCommand(new ShooterStop());
    //Constants.belt.setDefaultCommand(new BeltStop());
    driverXbox.a().onTrue(new IntakeUp());
    driverXbox.b().onTrue(new IntakeDown());
    //driverXbox.povUp().whileTrue(new IncreaseSpeed());
    //driverXbox.povDown().whileTrue(new DecreaseSpeed());

    driverXbox.rightTrigger().whileTrue(Constants.shooter.runShooter());
    // driverXbox.rightTrigger().whileFalse(new BeltStop());

  }
/*
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    String selectedAuto = m_chooser.getSelected();
    return Constants.drivebase.getAutonomousCommand(selectedAuto);

  }

  public void setDriveMode() {
    configureBindings();
  }

  public void setMotorBrake(boolean brake) {
    Constants.drivebase.setMotorBrake(false);
  }

  public void zeroEverything() {
    Constants.drivebase.zeroGyro();
  }

  /**
   * Replaces the swerve module feedforward with a new SimpleMotorFeedforward
   * object.
   *
   * @param kS the static gain of the feedforward
   * @param kV the velocity gain of the feedforward
   * @param kA the acceleration gain of the feedforward
   */

  public void setDriveFeedForward(double kS, double kV, double kA) {
    Constants.drivebase.replaceSwerveModuleFeedforward(kS, kV, kA);
  }

  public void resetAndStop() {
    Constants.drivebase.drive(new Translation2d(), 0, false);
  }

}

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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;

import swervelib.SwerveInputStream;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public final         CommandXboxController driverXbox = new CommandXboxController(0);
  public final         CommandXboxController operatorController = new CommandXboxController(1);

  //auto list object
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  //red or blue side object

  // The robot's subsystems and commands are defined here...
  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the rotational velocity 
  // buttons are quick rotation positions to different ways to face
  // WARNING: default buttons are on the same buttons as the ones defined in configureBindings
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
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
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
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(() -> -driverXbox.getRightX(),
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

  


  Command driveSetpointGen = Constants.drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);


  SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(Constants.drivebase.getSwerveDrive(),
                                                                   () -> driverXbox.getLeftY() * getSide(), //getSide will invert if on Red side
                                                                   () -> driverXbox.getLeftX() * getSide())
                                                               .withControllerRotationAxis(() -> -driverXbox.getRightX())
                                                               .deadband(Constants.OPERATOR_CONSTANTS.deadband())
                                                               .scaleTranslation(0.8)
                                                               .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleSim     = driveAngularVelocitySim.copy()
                                                                     .withControllerHeadingAxis(() -> Math.sin(
                                                                                                    driverXbox.getRawAxis(
                                                                                                        2) * Math.PI) * (Math.PI * 2),
                                                                                                () -> Math.cos(
                                                                                                    driverXbox.getRawAxis(
                                                                                                        2) * Math.PI) *
                                                                                                      (Math.PI * 2))
                                                                     .headingWhile(true);

  Command driveFieldOrientedDirectAngleSim = Constants.drivebase.driveFieldOriented(driveDirectAngleSim);
  Command driveFieldOrientedAngularVelocitySim = Constants.drivebase.driveFieldOriented(driveAngularVelocitySim);

  Command driveSetpointGenSim = Constants.drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleSim);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    //set up the shuffleboard tab
    Shuffleboard.getTab("Autonomous")
        .add("Auto Selector", m_chooser)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .withPosition(0, 0)
        .withSize(2, 1);
        

    Shuffleboard.getTab("Autonomous")
        .add("On Red Side?", true)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .getEntry();

    //set up auto and add options
    m_chooser.setDefaultOption("21", "21");
    m_chooser.addOption("1", "1");
    m_chooser.addOption("2", "2");
    m_chooser.addOption("3", "3");
    m_chooser.addOption("4", "4");
    m_chooser.addOption("5", "5");
    m_chooser.addOption("6", "6");
    m_chooser.addOption("7", "7");
    m_chooser.addOption("8", "8");
    m_chooser.addOption("9", "9");
    m_chooser.addOption("10", "10");
    m_chooser.addOption("11", "11");
    m_chooser.addOption("12", "12");
    m_chooser.addOption("13", "13");
    m_chooser.addOption("14", "14");
    m_chooser.addOption("15", "15");
    m_chooser.addOption("16", "16");
    m_chooser.addOption("17", "17");
    m_chooser.addOption("18", "18");
    m_chooser.addOption("19", "19");
    m_chooser.addOption("20", "20");
    m_chooser.addOption("22", "22");
    m_chooser.addOption("23", "23");
    m_chooser.addOption("24", "24");
    m_chooser.addOption("25", "25");
    m_chooser.addOption("26", "26");
    m_chooser.addOption("27", "27");
    m_chooser.addOption("28", "28");
    m_chooser.addOption("29", "29");
    m_chooser.addOption("30", "30");
    m_chooser.addOption("31", "31");
    m_chooser.addOption("32", "32");
    m_chooser.addOption("33", "33");
    m_chooser.addOption("34", "34");
    m_chooser.addOption("35", "35");
    m_chooser.addOption("36", "36");
    m_chooser.addOption("37", "37");
    m_chooser.addOption("38", "38");
    m_chooser.addOption("39", "39");

    m_chooser.addOption("41", "41");
    m_chooser.addOption("42", "42");
    m_chooser.addOption("43", "43");
    m_chooser.addOption("44", "44");
    m_chooser.addOption("45", "45");
    m_chooser.addOption("46", "46");
    m_chooser.addOption("47", "47");
    m_chooser.addOption("48", "48");

    m_chooser.addOption("50", "50");
    m_chooser.addOption("51", "51");
    m_chooser.addOption("52", "52");
    m_chooser.addOption("53", "53");
    m_chooser.addOption("54", "54");
    m_chooser.addOption("55", "55");
    m_chooser.addOption("56", "56");
    m_chooser.addOption("57", "57");
    m_chooser.addOption("58", "58");
    m_chooser.addOption("59", "59");
    m_chooser.addOption("60", "60");
    m_chooser.addOption("61", "61");
    m_chooser.addOption("62", "62");
    m_chooser.addOption("63", "63");
    m_chooser.addOption("64", "64");
    m_chooser.addOption("65", "65");
    m_chooser.addOption("66", "66");
    m_chooser.addOption("67", "67");
    m_chooser.addOption("68", "68");
    m_chooser.addOption("69", "69");
    m_chooser.addOption("2468", "2468");

    
    SmartDashboard.putData("Auto choices", m_chooser);
    
  
    
    // Initialize with proper alliance orientation
    Constants.drivebase.zeroGyroWithAlliance();
  }

  public int getSide(){

    if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red){
      return 1;
    }
    else{
      return -1;
    }

  }

  public void autoLock(){
    double leftX = driverXbox.getLeftX();
    double leftY = driverXbox.getLeftY();
    double rightX = driverXbox.getRightX();

    // if(leftX < Constants.OPERATOR_CONSTANTS.deadband() && leftY < Constants.OPERATOR_CONSTANTS.deadband() && rightX < Constants.OPERATOR_CONSTANTS.deadband()){
    //   Constants.lockTimer++;
    //   if(Constants.lockTimer > 25){
    //     Constants.drivebase.lock();
    //     Constants.lockTimer = 0;
    //   }
    // }

      SmartDashboard.putNumber("scale factor", Constants.scaleFactor);
      boolean rightTrigger = driverXbox.getRightTriggerAxis() > 0.3;
      if(rightTrigger){
        Constants.drivebase.lock();
      }
    }


  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // (Condition) ? Return-On-True : Return-on-False
    Constants.drivebase.setDefaultCommand(!RobotBase.isSimulation() ?
                                driveFieldOrientedAnglularVelocity :
                                driveFieldOrientedAngularVelocitySim);

    if (Robot.isSimulation())
    {
      driverXbox.start().onTrue(Commands.runOnce(() -> Constants.drivebase.resetOdometry(new Pose2d(2, 2, new Rotation2d()))));
    } else
    {
      // Create a default command for the shooter to always run the intake logic
  }
}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    String selectedAuto = m_chooser.getSelected();
    return Constants.drivebase.getAutonomousCommand(selectedAuto);

  }

  public void setDriveMode()
  {
    configureBindings();
  }

  public void setMotorBrake(boolean brake)
  {
    Constants.drivebase.setMotorBrake(false);
  }

  public void zeroEverything(){
    Constants.drivebase.zeroGyro();
  }
    /**
   * Replaces the swerve module feedforward with a new SimpleMotorFeedforward object.
   *
   * @param kS the static gain of the feedforward
   * @param kV the velocity gain of the feedforward
   * @param kA the acceleration gain of the feedforward
   */

  public void setDriveFeedForward(double kS, double kV, double kA){
    Constants.drivebase.replaceSwerveModuleFeedforward(kS, kV, kA);
  }

  public void resetAndStop(){
    Constants.drivebase.drive(new Translation2d(), 0, false);
  }
  
}

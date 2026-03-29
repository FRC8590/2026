// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.commands.Shoot;
import frc.robot.commands.StableShoot;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
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
    public final Swerve drivebase = new Swerve(
            new File(Filesystem.getDeployDirectory(), "swerve/neo"));
    public Vision vision = new Vision(() -> drivebase.getPose());
    public Belt belt = new Belt();
    public Intake intake = new Intake();
    public Shooter shooter = new Shooter();

    private final double deadband = 0.01;

    // Replace with CommandPS4Controller or CommandJoystick if needed
    public final CommandXboxController driverXbox = new CommandXboxController(0);
    public final CommandXboxController operatorController = new CommandXboxController(1);

    // auto list object
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    // Basically the sensitivity of the swerves
    public static double scaleFactor = 1;

    /**
     * Converts driver input into a field-relative ChassisSpeeds that is controlled
     * by angular velocity.
     */

    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
            () -> driverXbox.getLeftY() * getSide() * scaleFactor,
            () -> driverXbox.getLeftX() * getSide() * scaleFactor)
            .withControllerRotationAxis(() -> -driverXbox.getRightX() * 0.72 * scaleFactor)
            .deadband(deadband)
            .robotRelative(false)
            .allianceRelativeControl(false);

    SwerveInputStream driveRobotOriented = SwerveInputStream.of(drivebase.getSwerveDrive(),
            () -> -driverXbox.getLeftY() * scaleFactor,
            () -> -driverXbox.getLeftX() * scaleFactor)
            .withControllerRotationAxis(() -> -driverXbox.getRightX() * 0.6 * scaleFactor)
            .deadband(deadband)
            .robotRelative(true)
            .scaleTranslation(scaleFactor);

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
    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveRobotOrientedAngular = drivebase.driveRobotRelative(driveRobotOriented);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

    SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
            () -> -driverXbox.getLeftY() * getSide() * scaleFactor,
            () -> -driverXbox.getLeftX() * getSide() * scaleFactor)
            .withControllerRotationAxis(() -> -driverXbox.getRightX() * 0.72 * scaleFactor)
            .deadband(deadband)
            .robotRelative(false)
            .allianceRelativeControl(false);

    // Derive the heading axis with math!
    SwerveInputStream driveDirectAngleSim = driveAngularVelocitySim.copy()
            .withControllerHeadingAxis(() -> Math.sin(
                    driverXbox.getRawAxis(2) * Math.PI)
                    * (Math.PI * 2),
                    () -> Math.cos(
                            driverXbox.getRawAxis(2) * Math.PI)
                            * (Math.PI * 2))
            .headingWhile(true);

    Command driveFieldOrientedDirectAngleSim = drivebase.driveFieldOriented(driveDirectAngleSim);
    Command driveFieldOrientedAngularVelocitySim = drivebase.driveFieldOriented(driveAngularVelocitySim);

    Command driveSetpointGenSim = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleSim);

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

        SmartDashboard.putData("Auto choices", m_chooser);
        m_chooser.setDefaultOption("Do Nothing", "nada");
        // m_chooser.addOption("Sample Auto", "Sample Auto");
        m_chooser.addOption("Blue Left F.I.", "Blue-TrTo-7To");
        m_chooser.addOption("Blue Mid F.I.", "Blue-TrMd-7Md");
        m_chooser.addOption("Blue Right F.I.", "Blue-TrBo-7Bo");
        m_chooser.addOption("Red Right F.I.", "Red-TrTo-7To");
        m_chooser.addOption("Red Mid F.I.", "Red-TrMd-7Md");
        m_chooser.addOption("Red Left F.I.", "Red-TrBo-7Bo");
        m_chooser.addOption("Red Right Outpost", "Red-TrTo-Op-7To");
        m_chooser.addOption("Blue Left Outpost", "Blue-TrBo-Op-7Bo");

        // Initialize with proper alliance orientation
        NamedCommands.registerCommand("Shoot", new StableShoot());
        NamedCommands.registerCommand("IndexerRun", belt.indexerRun());
        NamedCommands.registerCommand("BeltRun", belt.beltRun());
        NamedCommands.registerCommand("IndexerStop", belt.indexerStop());
        NamedCommands.registerCommand("BeltStop", belt.beltStop());
        NamedCommands.registerCommand("BeltAndIndexerRun", belt.beltAndIndexerRun());
        NamedCommands.registerCommand("BeltAndIndexerStop", belt.beltAndIndexerStop());
        NamedCommands.registerCommand("IntakeDown", intake.intakeDown());

    }

    /**
     * Riley: Looks like another Joseff special over here. Appears to return 1 or -1
     * to invert controlls for red vs blue alliance
     * 
     * @return 1 if red side, -1 if blue side
     */
    public int getSide() {
        if (Systems.isRedAlliance()) {
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
        drivebase.setDefaultCommand(
                RobotBase.isSimulation()
                        ? driveFieldOrientedAngularVelocitySim
                        : driveFieldOrientedAnglularVelocity);

        driverXbox.povRight().onTrue(drivebase.shiftUp());
        driverXbox.povLeft().onTrue(drivebase.shiftDown());

        driverXbox.x().whileTrue(intake.intakeDown());
        driverXbox.b().whileTrue(intake.intakeUp());
        driverXbox.leftTrigger().whileTrue(intake.intakeDown());
        driverXbox.leftTrigger().whileTrue(intake.intakeRun());
        driverXbox.leftTrigger().whileFalse(intake.intakeStop());

        driverXbox.povUp().whileTrue(belt.beltAndIndexerRun());
        driverXbox.povUp().whileFalse(belt.beltAndIndexerStop());

        driverXbox.povDown().whileTrue(drivebase.lockPose());

        driverXbox.leftStick().whileTrue(drivebase.ZeroGryo());
        driverXbox.a().whileTrue(belt.beltRunReversed());
        driverXbox.a().whileFalse(belt.beltStop());
        driverXbox.a().whileTrue(belt.indexerRunReversed());
        driverXbox.a().whileFalse(belt.indexerStop());
        driverXbox.rightBumper().whileTrue(new StableShoot());
        driverXbox.rightBumper().whileFalse(shooter.shooterSetGoalRPM(0));
        driverXbox.rightBumper().whileFalse(belt.beltAndIndexerStop());
        driverXbox.rightTrigger().whileTrue(new Shoot());
        driverXbox.rightTrigger().whileFalse(shooter.shooterSetGoalRPM(0));
        driverXbox.rightTrigger().whileFalse(belt.beltAndIndexerStop());

        driverXbox.y().whileTrue(drivebase.aimAtTarget());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        String selectedAuto = m_chooser.getSelected();
        return drivebase.getAutonomousCommand(selectedAuto);

    }

    public void setMotorBrake(boolean brake) {
        drivebase.setMotorBrake(false);
    }

    public void zeroEverything() {
        drivebase.zeroGyro();
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
        drivebase.replaceSwerveModuleFeedforward(kS, kV, kA);
    }

    public void resetAndStop() {
        drivebase.drive(new Translation2d(), 0, false);
    }

}

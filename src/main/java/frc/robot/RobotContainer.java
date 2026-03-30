// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
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
import java.util.ArrayList;
import java.util.Arrays;

import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.commands.AimAtTarget;
import frc.robot.commands.RunIntake;
import frc.robot.commands.ZeroGyro;
import frc.robot.commands.feeder.Feed;
import frc.robot.commands.feeder.Unjam;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.shooter.StableShoot;
import frc.robot.services.vision.VisionService;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.drive.SimulatedSwerve;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.SimulatedIntake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.SimulatedShooter;
import swervelib.SwerveInputStream;
import lib.woodsonrobotics.SystemWrapper;
import lib.woodsonrobotics.vision.photon.PhotonVisionCamera;
import lib.woodsonrobotics.vision.photon.SimulatedPhotonVisionCamera;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {
    public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout
            .loadField(AprilTagFields.k2026RebuiltAndymark);

    private final PhotonVisionCamera[] ALL_CAMERAS = {
            PhotonVisionCamera.newArduCamera("front", fieldLayout, new Transform3d(
                    new Translation3d(
                            Units.inchesToMeters(-7.491),
                            Units.inchesToMeters(-8.427),
                            Units.inchesToMeters(17.923)),
                    new Rotation3d(Units.degreesToRadians(90), Units.degreesToRadians(29), 0)))
    };

    public final VisionService vision;
    public final SystemWrapper<? extends Swerve> drive;
    public final SystemWrapper<Shooter> shooter;
    public final SystemWrapper<Belt> belt;
    public final SystemWrapper<Indexer> indexer;
    public final SystemWrapper<Intake> intake;

    private final double deadband = 0.01;

    // Replace with CommandPS4Controller or CommandJoystick if needed
    public final CommandXboxController driverXbox = new CommandXboxController(0);
    public final CommandXboxController operatorController = new CommandXboxController(1);

    // auto list object
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    // Basically the sensitivity of the swerves
    public static double scaleFactor = 1;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // These don't have simulation variants
        belt = new SystemWrapper<>("belt", () -> new Belt());
        indexer = new SystemWrapper<>("indexer", () -> new Indexer());
        if (Robot.isReal()) {
            vision = new VisionService(ALL_CAMERAS);
            drive = new SystemWrapper<>("drive", () -> new Swerve(
                    new File(Filesystem.getDeployDirectory(), "swerve/neo"), vision));
            shooter = new SystemWrapper<>("shooter", () -> new Shooter());
            intake = new SystemWrapper<>("intake", () -> new Intake());
        } else {
            SimulatedPhotonVisionCamera[] simulated = Arrays.stream(ALL_CAMERAS)
                    .map(SimulatedPhotonVisionCamera::new)
                    .toArray(SimulatedPhotonVisionCamera[]::new);
            vision = new VisionService(simulated);
            SystemWrapper<SimulatedSwerve> simulatedDrive = new SystemWrapper<>("drive", () -> new SimulatedSwerve(
                    new File(Filesystem.getDeployDirectory(), "swerve/neo"), vision));
            drive = simulatedDrive;
            shooter = new SystemWrapper<>("shooter", () -> new SimulatedShooter());
            intake = new SystemWrapper<>("intake", () -> new SimulatedIntake(simulatedDrive));
        }

        configureBindings();

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
        NamedCommands.registerCommand("Shoot", new StableShoot(shooter, belt, indexer));

    }

    /**
     * Checks if the alliance is red, defaults to false if alliance isn't available.
     *
     * @return true if the red alliance, false if blue. Defaults to false if none is
     *         available.
     */
    public static boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
    }

    /**
     * Riley: Looks like another Joseff special over here. Appears to return 1 or -1
     * to invert controlls for red vs blue alliance
     * 
     * @return 1 if red side, -1 if blue side
     */
    private static int getSide() {
        if (isRedAlliance()) {
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
        Command driveFieldOrientedAnglularVelocity = drive
                .command(swerve -> swerve.driveFieldOriented(SwerveInputStream.of(swerve.getSwerveDrive(),
                        () -> driverXbox.getLeftY() * getSide() * scaleFactor,
                        () -> driverXbox.getLeftX() * getSide() * scaleFactor)
                        .withControllerRotationAxis(() -> -driverXbox.getRightX() * 0.72 * scaleFactor)
                        .deadband(deadband)
                        .robotRelative(false)
                        .allianceRelativeControl(false)));

        Command driveFieldOrientedAngularVelocitySim = drive
                .command(swerve -> swerve.driveFieldOriented(SwerveInputStream.of(swerve.getSwerveDrive(),
                        () -> -driverXbox.getLeftY() * getSide() * scaleFactor,
                        () -> -driverXbox.getLeftX() * getSide() * scaleFactor)
                        .withControllerRotationAxis(() -> -driverXbox.getRightX() * 0.72 * scaleFactor)
                        .deadband(deadband)
                        .robotRelative(false)
                        .allianceRelativeControl(false)));
        drive.setDefaultCommand(
                RobotBase.isSimulation()
                        ? driveFieldOrientedAngularVelocitySim
                        : driveFieldOrientedAnglularVelocity);

        driverXbox.povRight().onTrue(drive.command(Swerve::shiftUp));
        driverXbox.povLeft().onTrue(drive.command(Swerve::shiftDown));

        driverXbox.leftTrigger().whileTrue(new RunIntake(intake));

        driverXbox.povUp().whileTrue(new Feed(belt, indexer));

        driverXbox.povDown().whileTrue(drive.command(Swerve::lockPose));

        driverXbox.leftStick().whileTrue(new ZeroGyro(drive));
        driverXbox.a().whileTrue(new Unjam(belt, indexer));

        driverXbox.rightBumper().whileTrue(new StableShoot(shooter, belt, indexer));
        driverXbox.rightTrigger().whileTrue(new Shoot(shooter, belt, indexer, vision, drive));

        driverXbox.y().whileTrue(new AimAtTarget(vision, drive));

        if (Robot.isSimulation()) {
            DriverStation.silenceJoystickConnectionWarning(true);
        }
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        String selectedAuto = m_chooser.getSelected();
        return drive.command(swerve -> swerve.getAutonomousCommand(selectedAuto));
    }

    public void resetAndStop() {
        drive.ifEnabled(swerve -> swerve.drive(new Translation2d(), 0, false));
    }

}

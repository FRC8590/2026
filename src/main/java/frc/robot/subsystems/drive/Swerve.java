// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Meter;

import swervelib.SwerveModule;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.GenericEntry;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.services.vision.VisionService;
import lib.woodsonrobotics.telemetry.notify.DriveNotifier;

import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;
import edu.wpi.first.units.measure.Force;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import org.json.simple.parser.ParseException;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Swerve extends SubsystemBase {

    public static final double DEFAULT_SPEED = 5.0; // meters per second
    public static final double MAX_SPEED = 6.0; // meters per second

    /**
     * Swerve drive object.
     */
    protected final SwerveDrive swerveDrive;

    private double currentSpeed;
    private static final GenericEntry driveSpeedEntry = Shuffleboard.getTab("Drive")
            .add("Speed", 0)
            .withWidget(BuiltInWidgets.kDial)
            .withSize(1, 1)
            .withProperties(Map.of("min", 0, "max", MAX_SPEED))
            .withPosition(0, 0)
            .getEntry();
    private static final GenericEntry currentShiftEntry = Shuffleboard.getTab("Drive")
            .add("Target Speed", DEFAULT_SPEED)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withSize(1, 1)
            .withProperties(Map.of("min", 0, "max", MAX_SPEED))
            .withPosition(0, 1)
            .getEntry();

    private SwerveModule[] swerveModules;
    private GenericEntry[][] swerveEntries;
    private final Field2d field = new Field2d();

    private final VisionService visionService;

    private final void initShuffleboard() {
        swerveModules = swerveDrive.getModules();
        swerveEntries = new GenericEntry[swerveModules.length][2];

        ShuffleboardTab driveTab = Shuffleboard.getTab("Drive");

        ShuffleboardLayout moduleLayout = driveTab
                .getLayout("Modules", BuiltInLayouts.kGrid)
                .withPosition(1, 0).withSize(6, 3)
                .withProperties(Map.of("numberofcolumns", 4, "numberofrows", 1));
        for (int i = 0; i < swerveModules.length; i++) {

            ShuffleboardLayout currentLayout = moduleLayout
                    .getLayout(String.format("Module %1d", i), BuiltInLayouts.kGrid)
                    .withProperties(Map.of("numberofcolumns", 1, "numberofrows", 2));

            swerveEntries[i][1] = currentLayout
                    .add("Angle motor RPM", 0)
                    .withPosition(0, 1)
                    .getEntry();

            swerveEntries[i][0] = currentLayout
                    .add("Drive motor RPM", 0)
                    .withPosition(0, 0)
                    .getEntry();

        }

        Shuffleboard.getTab("Console")
                .add("Estimated Pose", field)
                .withSize(4, 2)
                .withWidget(BuiltInWidgets.kField);
    }

    /**
     * Initialize {@link SwerveDrive} with the directory provided.
     *
     * @param directory Directory of swerve drive config files.
     */
    public Swerve(File directory, VisionService vision) {
        visionService = vision;

        // Angle conversion factor is 360 / (GEAR RATIO * ENCODER RESOLUTION)
        // In this case the gear ratio is 12.8 motor revolutions per wheel rotation.
        // The encoder resolution per motor revolution is 1 per motor revolution.
        double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(12.8);
        // Motor conversion factor is (PI * WHEEL DIAMETER IN METERS) / (GEAR RATIO *
        // ENCODER RESOLUTION).
        // In this case the wheel diameter is 4 inches, which must be converted to
        // meters to get meters/second.
        // The gear ratio is 6.75 motor revolutions per wheel rotation.
        // The encoder resolution per motor revolution is 1 per motor revolution.
        double driveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(4), 6.75);

        currentSpeed = DEFAULT_SPEED;
        // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary
        // objects being created.
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.LOW;
        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(MAX_SPEED,
                    new Pose2d(new Translation2d(Meter.of(7.566),
                            Meter.of(6.19)),
                            Rotation2d.fromDegrees(180)));
            // Alternative method if you don't want to supply the conversion factor via JSON
            // files.
            // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed,
            // angleConversionFactor, driveConversionFactor);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        boolean isInSimulation = Robot.isSimulation();

        // Heading correction should only be used while controlling the robot
        // via angle.
        swerveDrive.setHeadingCorrection(false);
        swerveDrive.setCosineCompensator(isInSimulation);

        // Correct for skew that gets worse as angular velocity increases.
        // Start with a coefficient of 0.1.
        // We need to disable this when simulating because the simulation
        // overcompensates.
        swerveDrive.setAngularVelocityCompensation(!isInSimulation,
                !isInSimulation,
                0.1);

        // Enable if you want to resynchronize your absolute encoders and motor encoders
        // periodically when they are not moving.
        swerveDrive.setModuleEncoderAutoSynchronize(false, 1);

        // swerveDrive.pushOffsetsToEncoders(); // DEPRECATED, but might break things if
        // suggested replacement doesn't work
        swerveDrive.useExternalFeedbackSensor(); // we will see if this destroys things

        // Stop the odometry thread if we are using vision that way we can synchronize
        // updates better.
        swerveDrive.stopOdometryThread();

        initShuffleboard();

    }

    /**
     * Construct the swerve drive.
     *
     * @param driveCfg      SwerveDriveConfiguration for the swerve.
     * @param controllerCfg Swerve Controller.
     */
    public Swerve(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg,
            VisionService vision) {
        visionService = vision;
        initShuffleboard();
        currentSpeed = DEFAULT_SPEED;
        swerveDrive = new SwerveDrive(driveCfg,
                controllerCfg,
                MAX_SPEED,
                new Pose2d(
                        new Translation2d(
                                Meter.of(7.566),
                                Meter.of(6.19)),
                        Rotation2d.fromDegrees(180)));
    }

    private int telemetryCounter = 0;

    @Override
    public void periodic() {
        // When vision is enabled we must manually update odometry in SwerveDrive
        swerveDrive.updateOdometry();
        visionService.updateSwerveEstimation(swerveDrive);

        if (++telemetryCounter >= 5 || Robot.isSimulation()) {
            field.setRobotPose(swerveDrive.swerveDrivePoseEstimator.getEstimatedPosition());

            for (int i = 0; i < swerveModules.length; i++) {
                swerveEntries[i][0].setDouble(swerveModules[i].getDriveMotor().getVelocity() / 6);
                swerveEntries[i][1].setDouble(swerveModules[i].getAngleMotor().getVelocity() / 6);
            }
            telemetryCounter = 0;
        }
    }

    public Command lockPose() {
        return run(() -> {
            swerveDrive.lockPose();
        });
    }

    private void updateSpeed(ChassisSpeeds speeds) {
        driveSpeedEntry.setDouble(Math.abs(speeds.vxMetersPerSecond));
    }

    /**
     * Setup AutoBuilder for PathPlanner.
     */
    public void setupPathPlanner() {
        // Load the RobotConfig from the GUI settings. You should probably
        // store this in your Constants file
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();

            final boolean enableFeedforward = false;

            // Configure AutoBuilder last
            AutoBuilder.configure(
                    this::getPose,
                    // Robot pose supplier
                    this::resetOdometry,
                    // Method to reset odometry (will be called if your auto has a starting pose)
                    this::getRobotVelocity,
                    // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                    (speedsRobotRelative, moduleFeedForwards) -> {
                        if (enableFeedforward) {
                            swerveDrive.drive(
                                    speedsRobotRelative,
                                    swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                                    moduleFeedForwards.linearForces());
                            Force[] forces = moduleFeedForwards.linearForces();

                        } else {
                            swerveDrive.setChassisSpeeds(speedsRobotRelative);
                        }
                    },
                    // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also
                    // optionally outputs individual module feedforwards
                    new PPHolonomicDriveController(
                            // PPHolonomicController is the built in path following controller for holonomic
                            // drive trains
                            new PIDConstants(4.5, 0, 0),
                            // new PIDConstants(0.1, 0.0, 0.0)
                            // new PIDConstants(4, 0, 0.1),

                            new PIDConstants(2, 0, 0)

                    /*
                     * WORKING PID WITHOUT FF (disable FF to make it work)
                     * new PIDConstants(4, 0, 0.1),
                     * // Translation PID constants
                     * new PIDConstants(4, 0.0, 0.0)
                     * // Rotation PID constants
                     * 
                     */
                    ),
                    config,
                    // The robot configuration
                    () -> {
                        // Boolean supplier that controls when the path will be mirrored for the red
                        // alliance
                        // This will flip the path being followed to the red side of the field.
                        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                        // var alliance = DriverStation.getAlliance();
                        // if (alliance.isPresent())
                        // {
                        // return alliance.get() == DriverStation.Alliance.Red;
                        // }

                        // This was overwritten in the getSide() method of RobotContainer, so this part
                        // just defaults to false now

                        return false;
                    },
                    this
            // Reference to this subsystem to set requirements
            );

        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        // Preload PathPlanner Path finding
        // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
        CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
    }

    public double[][] getMotorTemperatures() {
        SwerveModule[] modules = swerveDrive.getModules();

        return new double[][] {
                getModuleTemperature(modules[0]),
                getModuleTemperature(modules[1]),
                getModuleTemperature(modules[2]),
                getModuleTemperature(modules[3]) };
    }

    public static double[] getModuleTemperature(SwerveModule module) {
        SparkMax angleMotor = (SparkMax) (module.getAngleMotor().getMotor());
        SparkMax driveMotor = (SparkMax) (module.getDriveMotor().getMotor());

        return new double[] { angleMotor.getMotorTemperature(), driveMotor.getMotorTemperature() };
    }

    /**
     * Get the path follower with events.
     *
     * @param pathName PathPlanner path name.
     * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
     */
    public Command getAutonomousCommand(String pathName) {
        // Create a path following command using AutoBuilder. This will also trigger
        // event markers.
        return new PathPlannerAuto(pathName);
    }

    // TODO: Peter: Why is this unused?
    /**
     * Returns a Command that centers the modules of the SwerveDrive subsystem.
     *
     * @return a Command that centers the modules of the SwerveDrive subsystem
     */
    public Command centerModulesCommand() {
        return run(() -> Arrays.asList(swerveDrive.getModules())
                .forEach(it -> it.setAngle(0.0)));
    }

    /**
     * Returns a Command that drives the swerve drive to a specific distance at a
     * given speed.
     *
     * @param distanceInMeters       the distance to drive in meters
     * @param speedInMetersPerSecond the speed at which to drive in meters per
     *                               second
     * @return a Command that drives the swerve drive to a specific distance at a
     *         given speed
     */
    // public Command driveToDistanceCommand(double distanceInMeters, double
    // speedInMetersPerSecond)
    // {
    // return run(() -> drive(new ChassisSpeeds(speedInMetersPerSecond, 0, 0)))
    // .until(() -> swerveDrive.getPose().getTranslation().getDistance(new
    // Translation2d(0, 0)) >
    // distanceInMeters);
    // }

    public Command driveToDistanceCommand(double distanceInMeters, double speedInMetersPerSecond) {

        Pose2d startingPose2d = swerveDrive.getPose();

        return run(() -> drive(new ChassisSpeeds(speedInMetersPerSecond, 0, 0)))
                .until(() -> swerveDrive.getPose().getTranslation()
                        .getDistance(startingPose2d.getTranslation()) > distanceInMeters)
                .andThen(() -> lock());
    }

    /**
     * Replaces the swerve module feedforward with a new SimpleMotorFeedforward
     * object.
     *
     * @param kS the static gain of the feedforward
     * @param kV the velocity gain of the feedforward
     * @param kA the acceleration gain of the feedforward
     */
    public void replaceSwerveModuleFeedforward(double kS, double kV, double kA) {
        swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(kS, kV, kA));
    }

    /**
     * The primary method for controlling the drive. Takes a
     * {@link Translation2d} and a rotation rate, and
     * calculates and commands module states accordingly. Can use either open-loop
     * or closed-loop velocity control for
     * the wheel velocities. Also has field- and robot-relative modes, which affect
     * how the translation vector is used.
     *
     * @param translation   {@link Translation2d} that is the commanded linear
     *                      velocity of the robot, in meters per
     *                      second. In robot-relative mode, positive x is torwards
     *                      the bow (front) and positive y is
     *                      torwards port (left). In field-relative mode, positive x
     *                      is away from the alliance wall
     *                      (field North) and positive y is torwards the left wall
     *                      when looking through the driver station
     *                      glass (field West).
     * @param rotation      Robot angular rate, in radians per second. CCW positive.
     *                      Unaffected by field/robot
     *                      relativity.
     * @param fieldRelative Drive mode. True for field-relative, false for
     *                      robot-relative.
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        swerveDrive.drive(translation,
                rotation,
                fieldRelative,
                false); // Open loop is disabled since it shouldn't be used most of the time.

    }

    private double idleStartTime = -1;
    private static final double LOCK_DELAY_SECONDS = 0.3;

    /**
     * Drive the robot given a chassis field oriented velocity.
     *
     * @param velocity Velocity according to the field.
     */
    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
        return run(() -> {
            ChassisSpeeds speeds = velocity.get();
            updateSpeed(speeds);
            boolean isIdle = Math.abs(speeds.vxMetersPerSecond) < 0.05 &&
                    Math.abs(speeds.vyMetersPerSecond) < 0.05 &&
                    Math.abs(speeds.omegaRadiansPerSecond) < 0.05;

            // For defensive purposes, we want to lock the wheels when we're not
            // moving. This makes it very difficult to push us.
            if (isIdle) {
                if (idleStartTime < 0) {
                    idleStartTime = Timer.getFPGATimestamp();
                }
                if (Timer.getFPGATimestamp() - idleStartTime >= LOCK_DELAY_SECONDS) {
                    swerveDrive.lockPose();
                } else {
                    swerveDrive.driveFieldOriented(speeds);
                }
            } else {
                idleStartTime = -1;
                swerveDrive.driveFieldOriented(speeds);
            }
        });
    }

    /**
     * Drive the robot given a chassis field oriented velocity.
     *
     * @param velocity Velocity according to the field.
     */
    public void driveRobotRelative(ChassisSpeeds velocity) {
        updateSpeed(velocity);
        swerveDrive.drive(velocity);

    }

    /**
     * Drive the robot given a chassis field oriented velocity.
     *
     * @param velocity Velocity according to the field.
     */
    public Command driveRobotRelative(Supplier<ChassisSpeeds> velocity) {
        return run(() -> {
            ChassisSpeeds speeds = velocity.get();
            updateSpeed(speeds);
            swerveDrive.drive(speeds);
        });
    }

    /**
     * Drive according to the chassis robot oriented velocity.
     *
     * @param velocity Robot oriented {@link ChassisSpeeds}
     */
    public void drive(ChassisSpeeds velocity) {
        updateSpeed(velocity);
        swerveDrive.drive(velocity);
    }

    /* Increase the current speed. */
    public Command shiftUp() {
        return runOnce(() -> {
            DriveNotifier.inform("Shifting up speed");
            if (currentSpeed == MAX_SPEED) {
                DriveNotifier.operatorError("Swerve is already at max speed, cannot accelerate");
            } else {
                currentShiftEntry.setDouble(++currentSpeed);
                swerveDrive.setMaximumAllowableSpeeds(currentSpeed, swerveDrive.getMaximumChassisAngularVelocity());
            }
        });
    }

    /* Decrease the current speed. */
    public Command shiftDown() {
        return runOnce(() -> {
            assert (currentSpeed > 0);
            DriveNotifier.inform("Shifting down speed");
            if (currentSpeed == 1) {
                DriveNotifier.operatorError("Cannot make the swerves any slower");
            } else {
                currentShiftEntry.setDouble(--currentSpeed);
                swerveDrive.setMaximumAllowableSpeeds(currentSpeed, swerveDrive.getMaximumChassisAngularVelocity());
            }
        });
    }

    /**
     * Resets odometry to the given pose. Gyro angle and module positions do not
     * need to be reset when calling this
     * method. However, if either gyro angle or module position is reset, this must
     * be called in order for odometry to
     * keep working.
     *
     * @param initialHolonomicPose The pose to set the odometry to
     */
    public void resetOdometry(Pose2d initialHolonomicPose) {
        swerveDrive.resetOdometry(initialHolonomicPose);
    }

    /**
     * Gets the current pose (position and rotation) of the robot, as reported by
     * odometry.
     *
     * @return The robot's pose
     */
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    /**
     * Set chassis speeds with closed-loop velocity control.
     *
     * @param chassisSpeeds Chassis Speeds to set.
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        swerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    /**
     * Resets the gyro angle to zero and resets odometry to the same position, but
     * facing toward 0.
     */
    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }

    /**
     * Zero the gyro and properly initialize field-relative orientation based on
     * alliance.
     * This ensures compatibility with PathPlanner's coordinate system.
     */
    public void zeroGyroWithAlliance() {
        // First zero the gyro
        zeroGyro();

        // Get the current alliance
        if (RobotContainer.isRedAlliance()) {
            // When on red alliance, we need to rotate the field coordinate system 180
            // degrees
            // This matches PathPlanner's coordinate system where the origin stays on blue
            // side
            swerveDrive.setGyroOffset(new Rotation3d(0, 0, 180));
        }
    }

    /**
     * Sets the drive motors to brake/coast mode.
     *
     * @param brake True to set motors to brake mode, false for coast.
     */
    public void setMotorBrake(boolean brake) {
        swerveDrive.setMotorIdleMode(brake);
        swerveDrive.getModules()[0].getDriveMotor().burnFlash();
        swerveDrive.getModules()[1].getDriveMotor().burnFlash();
        swerveDrive.getModules()[2].getDriveMotor().burnFlash();
        swerveDrive.getModules()[3].getDriveMotor().burnFlash();
    }

    /**
     * Gets the current yaw angle of the robot, as reported by the swerve pose
     * estimator in the underlying drive.
     * Note, this is not the raw gyro reading, this may be corrected from calls to
     * resetOdometry().
     *
     * @return The yaw angle
     */
    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    /**
     * Get the chassis speeds based on controller input of 2 joysticks. One for
     * speeds in which direction. The other for
     * the angle of the robot.
     *
     * @param xInput   X joystick input for the robot to move in the X direction.
     * @param yInput   Y joystick input for the robot to move in the Y direction.
     * @param headingX X joystick which controls the angle of the robot.
     * @param headingY Y joystick which controls the angle of the robot.
     * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
     */
    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
        Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
        return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                scaledInputs.getY(),
                headingX,
                headingY,
                getHeading().getRadians(),
                currentSpeed);
    }

    /**
     * Get the chassis speeds based on controller input of 1 joystick and one angle.
     * Control the robot at an offset of
     * 90deg.
     *
     * @param xInput X joystick input for the robot to move in the X direction.
     * @param yInput Y joystick input for the robot to move in the Y direction.
     * @param angle  The angle in as a {@link Rotation2d}.
     * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
     */
    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
        Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

        return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                scaledInputs.getY(),
                angle.getRadians(),
                getHeading().getRadians(),
                currentSpeed);
    }

    /**
     * Gets the current field-relative velocity (x, y and omega) of the robot
     *
     * @return A ChassisSpeeds object of the current field-relative velocity
     */
    public ChassisSpeeds getFieldVelocity() {
        return swerveDrive.getFieldVelocity();
    }

    /**
     * Gets the current velocity (x, y and omega) of the robot
     *
     * @return A {@link ChassisSpeeds} object of the current velocity
     */
    public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
    }

    /**
     * Get the {@link SwerveDriveConfiguration} object.
     *
     * @return The {@link SwerveDriveConfiguration} fpr the current drive.
     */
    public SwerveDriveConfiguration getSwerveDriveConfiguration() {
        return swerveDrive.swerveDriveConfiguration;
    }

    /**
     * Lock the swerve drive to prevent it from moving.
     */
    public void lock() {
        DriveNotifier.inform("Swerve locked");
        swerveDrive.lockPose();
    }

    /**
     * Gets the current pitch angle of the robot, as reported by the imu.
     *
     * @return The heading as a {@link Rotation2d} angle
     */
    public Rotation2d getPitch() {
        return swerveDrive.getPitch();
    }

    public Command ZeroGryo() {
        return run(() -> zeroGyroWithAlliance());
    }

    /**
     * Gets the swerve drive object.
     *
     * @return {@link SwerveDrive}
     */
    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }

    /**
     * Drive with {@link SwerveSetpointGenerator} from 254, implemented by
     * PathPlanner.
     *
     * @param robotRelativeChassisSpeed Robot relative {@link ChassisSpeeds} to
     *                                  achieve.
     * @return {@link Command} to run.
     * @throws IOException    If the PathPlanner GUI settings is invalid
     * @throws ParseException If PathPlanner GUI settings is nonexistent.
     */
    private Command driveWithSetpointGenerator(Supplier<ChassisSpeeds> robotRelativeChassisSpeed)
            throws IOException, ParseException {
        SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator(RobotConfig.fromGUISettings(),
                swerveDrive.getMaximumChassisAngularVelocity());
        AtomicReference<SwerveSetpoint> prevSetpoint = new AtomicReference<>(
                new SwerveSetpoint(swerveDrive.getRobotVelocity(),
                        swerveDrive.getStates(),
                        DriveFeedforwards.zeros(swerveDrive.getModules().length)));
        AtomicReference<Double> previousTime = new AtomicReference<>();

        return startRun(() -> previousTime.set(Timer.getFPGATimestamp()),
                () -> {

                    double newTime = Timer.getFPGATimestamp();
                    SwerveSetpoint newSetpoint = setpointGenerator.generateSetpoint(prevSetpoint.get(),
                            robotRelativeChassisSpeed.get(),
                            newTime - previousTime.get());
                    ChassisSpeeds speeds = newSetpoint.robotRelativeSpeeds();
                    updateSpeed(speeds);
                    swerveDrive.drive(speeds,
                            newSetpoint.moduleStates(),
                            newSetpoint.feedforwards().linearForces());
                    prevSetpoint.set(newSetpoint);
                    previousTime.set(newTime);
                });
    }

    /**
     * Drive with 254's Setpoint generator; port written by PathPlanner.
     *
     * @param fieldRelativeSpeeds Field-Relative {@link ChassisSpeeds}
     * @return Command to drive the robot using the setpoint generator.
     */
    public Command driveWithSetpointGeneratorFieldRelative(Supplier<ChassisSpeeds> fieldRelativeSpeeds) {
        try {
            return driveWithSetpointGenerator(() -> {
                return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds.get(), getHeading());
            });
        } catch (Exception e) {
            DriverStation.reportError(e.toString(), true);
        }
        return Commands.none();

    }
}

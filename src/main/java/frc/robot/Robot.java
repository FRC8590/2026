// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import lib.woodsonrobotics.telemetry.ConsoleCountdown;
import lib.woodsonrobotics.telemetry.notify.DriveNotifier;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this
 * class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot {

    private static Robot instance;

    private Command autonomousCommand;
    private static GenericEntry isRedAllianceEntry = Shuffleboard
            .getTab("Console")
            .add("On red alliance?", false)
            .getEntry();

    public RobotContainer robotContainer;

    private Timer disabledTimer;

    private final double WHEEL_LOCK_TIME = 10.0;

    // Rebuilt-specific; time until the hub switches
    private ConsoleCountdown allianceShiftCountdown = new ConsoleCountdown("Time until shift");
    private int allianceShiftCounter = 0;
    private ConsoleCountdown timeUntilEnd = new ConsoleCountdown("Time until end");

    public final CommandXboxController driverXbox = new CommandXboxController(0);

    public Robot() {
        instance = this;
    }

    public static Robot getInstance() {
        return instance;
    }

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our
        // autonomous chooser on the dashboard.

        robotContainer = new RobotContainer();

        // Create a timer to disable motor brake a few seconds after disable. This will
        // let the robot stop
        // immediately when disabled, but then also let it be pushed more
        disabledTimer = new Timer();

        robotContainer.drive.ifEnabled(swerve -> {
            swerve.replaceSwerveModuleFeedforward(.0002, 2.8, 0);
            swerve.setupPathPlanner();
        });

        robotContainer.vision.startVisionThread();
        robotContainer.shooter.ifEnabled(shooter -> shooter.initialize());
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items
     * like diagnostics that you want ran
     * during disabled, autonomous, teleoperated and test.
     * 
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();

    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
        DriveNotifier.inform("Robot disabled");
        disabledTimer.reset();
        disabledTimer.start();

    }

    @Override
    public void disabledPeriodic() {
        if (disabledTimer.hasElapsed(WHEEL_LOCK_TIME)) {
            disabledTimer.stop();
        }
        robotContainer.resetAndStop();
    }

    private void ensureIntakeHomed() {
        robotContainer.intake.ifEnabled(intake -> {
            if (!intake.isHomed()) {
                CommandScheduler.getInstance().schedule(intake.homeCommand());
            }
        });
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        isRedAllianceEntry.setBoolean(RobotContainer.isRedAlliance());
        robotContainer.drive.ifEnabled(swerve -> swerve.zeroGyroWithAlliance());
        ensureIntakeHomed();

        autonomousCommand = robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }

        allianceShiftCountdown.start(20);
        timeUntilEnd.start(160);
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        // Constants.SHOOTER.processIntakeCoralAuto();
        timeUntilEnd.step();
    }

    @Override
    public void teleopInit() {
        DriveNotifier.inform("Robot in teleop");
        ensureIntakeHomed();

        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            DriveNotifier.inform("Cancelling auto command");
            autonomousCommand.cancel();
        }

        timeUntilEnd.start(140);
        allianceShiftCountdown.start(10); // Transition shift
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        double remainingTime = allianceShiftCountdown.step();
        if (remainingTime <= 0) {
            if (++allianceShiftCounter > 4) {
                allianceShiftCountdown.start(30);
            } else {
                allianceShiftCountdown.start(25);
            }
        }

        timeUntilEnd.step();

        // Controller Diagnostics
        SmartDashboard.putNumberArray("Controller Values", new Double[] { driverXbox.getLeftX(), driverXbox.getLeftY(),
                driverXbox.getRightX(), driverXbox.getRightY() });

    }

    /**
     * This function is called once when the robot is first started up.
     */
    @Override
    public void simulationInit() {
        robotContainer.simulation.simulationInit();
    }

    /**
     * This function is called periodically whilst in simulation.
     */
    @Override
    public void simulationPeriodic() {
        robotContainer.simulation.simulationPeriodic();
    }
}

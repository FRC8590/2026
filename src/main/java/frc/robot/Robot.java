// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.ironmaple.simulation.SimulatedArena;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import lib.woodsonrobotics.ConsoleCountdown;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this
 * class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot {

    private static Robot instance;

    private Command m_autonomousCommand;
    private GenericEntry isRedAllianceEntry = Shuffleboard
            .getTab("Console")
            .add("On red alliance?", false)
            .getEntry();

    public RobotContainer m_robotContainer;

    private Timer disabledTimer;

    private final double wheelLockTime = 10.0;

    // Rebuilt-specific; time until the hub switches
    private ConsoleCountdown allianceShiftCountdown = new ConsoleCountdown("Time until shift");
    private ConsoleCountdown timeUntilEnd = new ConsoleCountdown("Time until end");

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

        m_robotContainer = new RobotContainer();

        // Create a timer to disable motor brake a few seconds after disable. This will
        // let the robot stop
        // immediately when disabled, but then also let it be pushed more
        disabledTimer = new Timer();

        m_robotContainer.setDriveFeedForward(.0002, 2.8, 0);

        m_robotContainer.drivebase.setupPathPlanner();
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
        m_robotContainer.vision.updateVisionField();
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
        System.out.println("Robot disabled");
        disabledTimer.reset();
        disabledTimer.start();

    }

    @Override
    public void disabledPeriodic() {
        if (disabledTimer.hasElapsed(wheelLockTime)) {
            disabledTimer.stop();
        }
        m_robotContainer.resetAndStop();
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        isRedAllianceEntry.setBoolean(Systems.isRedAlliance());
        m_robotContainer.drivebase.zeroGyroWithAlliance();
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }

        timeUntilEnd.start(20);
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
        System.out.println("Robot in teleop");
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        System.out.println(m_autonomousCommand);
        if (m_autonomousCommand != null) {
            System.out.println("Cancelling auto command");
            m_autonomousCommand.cancel();
        } else {
            // System.out.println("Cancelling all commands");
            // System.out.println(CommandScheduler.getInstance());
            // CommandScheduler.getInstance().cancelAll();
        }

        allianceShiftCountdown.start(10);
        timeUntilEnd.start(140 /* 2:20 minutes */);
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        double remainingTime = allianceShiftCountdown.step();
        if (remainingTime <= 0) {
            allianceShiftCountdown.start(25);
        }

        timeUntilEnd.step();
    }

    /**
     * This function is called once when the robot is first started up.
     */
    @Override
    public void simulationInit() {
    }

    /**
     * This function is called periodically whilst in simulation.
     */
    @Override
    public void simulationPeriodic() {
        SimulatedArena.getInstance().simulationPeriodic();
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

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

    private class Countdown {
        private Timer timer;
        private double waitTime = -1;
        private GenericEntry shuffleboardEntry;

        public Countdown(String name) {
            timer = new Timer();
            shuffleboardEntry = Shuffleboard
                    .getTab("Console").add(name, 0).getEntry();
        }

        public void start(double waitTime) {
            this.waitTime = waitTime;
            timer.reset();
            timer.start();
            shuffleboardEntry.setDouble(waitTime);
        }

        public void stop() {
            timer.stop();
        }

        public double step() {
            double remainingTime = waitTime - timer.get();
            if (remainingTime <= 0) {
                stop();
                shuffleboardEntry.setInteger(0);
            } else {
                shuffleboardEntry.setDouble(Math.round(remainingTime));
            }
            return remainingTime;
        }
    }

    // Rebuilt-specific; time until the hub switches
    private Countdown allianceShiftCountdown;
    private Countdown timeUntilEnd;

    public Robot() {
        instance = this;
        allianceShiftCountdown = new Countdown("Time until shift");
        timeUntilEnd = new Countdown("Time until end");

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

        // Constants.visionTimerOffset =
        // Vision.Cameras.LEFT_CAM.resultsList.get(0).getTimestampSeconds();

        m_robotContainer.setDriveFeedForward(.0002, 2.8, 0);

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
        Constants.vision.updateVisionField();

        // SmartDashboard.putBoolean("right camrea status",
        // Constants.vision.getEnabled(1));
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
        if (disabledTimer.hasElapsed(Constants.DRIVE_CONSTANTS.wheelLockTime())) {
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
        System.out.println("Robot in auto");

        isRedAllianceEntry.setBoolean(Systems.isRedAlliance());
        System.out.println("gyro start");
        Constants.drivebase.zeroGyroWithAlliance();
        System.out.println("gyro calibrated");
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
    }
}

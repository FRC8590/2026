package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Systems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu .wpi.first.wpilibj2.command.Command;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.ResetMode;

import static edu.wpi.first.units.Units.Inches;

import java.util.Map;

import com.revrobotics.PersistMode;

public class Shooter extends SubsystemBase {
    private final SparkFlex frontMotor = new SparkFlex(Constants.SHOOTER_CONSTANTS.frontMotorID(),
            MotorType.kBrushless);
    private final SparkFlex backMotor = new SparkFlex(Constants.SHOOTER_CONSTANTS.backMotorID(), MotorType.kBrushless);

    private final SparkFlexConfig shooterConfig = new SparkFlexConfig();

    /** cV: cruiseVelocity. mA: maxAcceleration */
    private double goalRPM = 0, p, i, d, kA, kV, cV, mA;

    private GenericEntry targRPMEntry;
    private GenericEntry currRPMFrontEntry;
    private GenericEntry currRPMBackEntry;

    public Shooter() {
        p = 0.0001;
        i = 0;
        d = 0;
        kA = 0;
        kV = 0.0019;
        cV = 6700;
        mA = 4000; // todo increase

        shooterConfig // configure motors
                .inverted(false)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(60)
                .closedLoopRampRate(0.001); // todo look at this
        shooterConfig.closedLoop // configure PID
                .pid(p, i, d);
        shooterConfig.closedLoop.feedForward
                .kA(kA)
                .kV(kV);
        shooterConfig.closedLoop.maxMotion // configure maxMotion
                .cruiseVelocity(cV)
                .maxAcceleration(mA);

        frontMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        shooterConfig.inverted(true);
        backMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        SmartDashboard.putNumber("Shooter Speed", goalRPM);

        // Shuffleboard setup
        ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");

        ShuffleboardLayout shooterLayout = shooterTab.getLayout("Shooter", BuiltInLayouts.kGrid).withSize(2, 2)
                .withPosition(4, 0);
        SimpleWidget currRPMFrontWidget = shooterLayout.add("Front motor RPM", 0).withWidget(BuiltInWidgets.kDial)
                .withProperties(Map.of("Min", 0, "Max", Constants.SHOOTER_MAX_RPM)).withPosition(0, 1);
        SimpleWidget currRPMBackWidget = shooterLayout.add("Back motor RPM", 0).withWidget(BuiltInWidgets.kDial)
                .withProperties(Map.of("Min", 0, "Max", Constants.SHOOTER_MAX_RPM)).withPosition(1, 1);
        SimpleWidget targRPMWidget = shooterLayout.add("Target RPM", 0).withWidget(BuiltInWidgets.kDial)
                .withProperties(Map.of("min", 0, "max", Constants.SHOOTER_MAX_RPM)).withPosition(0, 0);

        currRPMFrontEntry = currRPMFrontWidget.getEntry();
        currRPMBackEntry = currRPMBackWidget.getEntry();

        targRPMEntry = targRPMWidget.getEntry();

    }

    private void setGoalRPM(double rpm) {
        targRPMEntry.setDouble(rpm);
        goalRPM = rpm;
    }

    public double distanceToRPM() {
        return 0;
    }

    /**
     * If the average RPM of the front and back shooter motors is at or above the
     * goal RPM
     * 
     * @return true if average RPM is at or above goal RPM
     */
    public boolean atRPM() {
        return (frontMotor.getEncoder().getVelocity() + backMotor.getEncoder().getVelocity()) / 2 >= goalRPM - 100;
    }

    /**
     * Change the goal RPM of the shooter. If rpm is greater than 6784, goal RPM is
     * not changed.
     * 
     * @param rpm rotations per minute you want the shooter to run at
     * @return command that sets the goal RPM
     */
    public Command shooterSetGoalRPM(double rpm) {
        if (rpm > Constants.SHOOTER_MAX_RPM)
            return run(() -> setGoalRPM(goalRPM));
        return run(() -> {
            if (Robot.isSimulation()) {
                // TODO: Peter: Tune this based on the regression model
                Constants.vision.simulateShoot(8, 29);
            }
            setGoalRPM(rpm);
        });
    }

    public Command shooterSetRPMFromVision() {
        return run(() -> {
            int primaryId = Constants.drivebase.isRedAlliance() ? 10 : 26;
            var result = Constants.vision.getBestSingleTagPoseEstimate(primaryId);
            if (!result.isPresent()) {
                // Nothing seen -- hope for the best!
                setGoalRPM(2000);
                return;
            }

            var distance = result.get().getMeasureX();
            // This is based off our regression model
            var rpm = 7.02381 * (distance.in(Inches)) + 1237.14286;
            setGoalRPM(rpm);
        });
    }

    /**
     * Sets the goal RPM to zero
     * 
     * @return command that sets the goal RPM to 0
     */
    public Command shooterStop() {
        return run(() -> setGoalRPM(0));
    }

    /**
     * This method will be called once per scheduler run
     * Put values you want to moniter here
     */
    @Override
    public void periodic() {
        if (Systems.isSystemEnabled(Systems.enableShooter)) {
            frontMotor.getClosedLoopController().setSetpoint(goalRPM, SparkBase.ControlType.kMAXMotionVelocityControl);
            backMotor.getClosedLoopController().setSetpoint(goalRPM, SparkBase.ControlType.kMAXMotionVelocityControl);
        }

        goalRPM = SmartDashboard.getNumber("Shooter Speed", goalRPM);
        SmartDashboard.putNumber("Set Shooter Speed ", goalRPM);
        SmartDashboard.putNumber("Front Motor RPM ", frontMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Back Motor RPM ", backMotor.getEncoder().getVelocity());
        targRPMEntry.setDouble(goalRPM);
        currRPMBackEntry.setDouble(backMotor.getEncoder().getVelocity());
        currRPMFrontEntry.setDouble(frontMotor.getEncoder().getVelocity());
    }
}

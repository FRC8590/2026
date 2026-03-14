package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;

public class Intake extends SubsystemBase {
    /**
     * This refers to how many rotations the pivotMotor must do
     * before intake does a (hypithetical) full rotation
     * 
     * @see 100:15 ratio, subject to change
     */

    // public so we can monitor the pivot motor in the dashboard.
    public final SparkMax pivotMotor = new SparkMax(Constants.INTAKE_CONSTANTS.pivotMotorID(), MotorType.kBrushless);
    private final SparkMax intakeMotor = new SparkMax(Constants.INTAKE_CONSTANTS.intakeMotorID(), MotorType.kBrushless);

    private final SparkMaxConfig intakeConfig = new SparkMaxConfig();
    private final SparkMaxConfig pivotConfig = new SparkMaxConfig();

    /** Relative Encoder */
    private final RelativeEncoder encoder;
    /** PID */
    private double p = 0.4;
    private double i = 0;
    private double d = 0;
    /** FeedForward */
    private double kv = 0.1;
    private double kcos = 0.45;
    private double kcosratio = 1;
                                    // also kcos messing things up
    private double goalUpRadians = 0.51;
    private double goalDownRadians = -0.05;
    private double setPoint = 0.7; // up position is ~0.7, but 0.5 to prevent it trying to go into the hopper,


    public Intake() {
        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // configure intake motor
        intakeConfig
                .inverted(true)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(20)
                .closedLoopRampRate(0.001);
        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // configure pivot motor
        pivotConfig
                .inverted(true)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(60)
                .closedLoopRampRate(0.001).closedLoop
                .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder); // slot 0
        // configure PID for the closed loop controller
        pivotConfig.closedLoop
                .pid(p, i, d).feedForward
                .kV(kv)
                .kCos(kcos)
                .kCosRatio(kcosratio);
        // .kS(ks)
        // .kA(ka)
        // .kG(0)
        // configure constrainsts(?) for maxMotion
        pivotConfig.closedLoop.maxMotion
                // Set MAXMotion parameters for position control. We don't need to pass
                // a closed loop slot, as it will default to slot 0.
                .cruiseVelocity(120)
                .maxAcceleration(20)
                .allowedProfileError(1);
        // configure encoder
        pivotConfig.alternateEncoder
                .setSparkMaxDataPortConfig();

        pivotMotor.configure(pivotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        encoder = pivotMotor.getAlternateEncoder();
        encoder.setPosition(0.744); // zero encoder, such that the down position is 0 and the up position is 0.7
    }

    /**
     * 
     * = = = = = = = = = = METHODS = = = = = = = = = =
     * 
     */

    /**
     * Set the intake up or down
     * 
     * @param pointSet rotation, in radians at the encoder, that the pivot motor
     *                 should go to. 0 is down, 0.5 is up, starts at 0.744 when all
     *                 the way back
     */
    private void setGoal(double pointSet) {
        setPoint = pointSet;
    }

    private void up() {
        //intakeMotor.set(0);
        setGoal(goalUpRadians);
    }


    private void down() {
        // intakeMotor.set(1);
        setGoal(goalDownRadians);
    }
    private void run()
    {
       intakeMotor.set(1); 
    }
    private void stop()
    {
       intakeMotor.set(0); 
    }

    public Command intakeRun() {
        return run(() -> run());
    }
    public Command intakeStop() {
        return run(() -> stop());
    }
    /**
     * pivot up the and stop the intake
     * 
     * @return runnable Command
     */
    public Command intakeUp() {
        return run(() -> up());
    }

    /**
     * pivot down and start the intake
     * 
     * @return runnable Command
     */
    public Command intakeDown() {
        return run(() -> down());
    }

    @Override
    public void periodic() {
        if (Constants.ennableIntakeArm)
            pivotMotor.getClosedLoopController().setSetpoint(setPoint, SparkBase.ControlType.kMAXMotionPositionControl);
    }
}
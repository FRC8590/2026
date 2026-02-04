package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;

import java.util.concurrent.Callable;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;

public class IntakeSubsystem extends SubsystemBase {
    /**
     * This refers to how many rotations the pivotMotor must do
     * before intake does a (hypithetical) full rotation
     * 
     * @see 100:15 ratio, subject to change
     */
    private final double pivotRatio = 100 / 15;

    private final SparkMax pivotMotor = new SparkMax(Constants.INTAKE_CONSTANTS.pivotMotorID(), MotorType.kBrushless);
    private final SparkMax intakeMotor = new SparkMax(Constants.INTAKE_CONSTANTS.intakeMotorID(), MotorType.kBrushless);

    private final SparkMaxConfig intakeConfig = new SparkMaxConfig();
    private final SparkMaxConfig pivotConfig = new SparkMaxConfig();

    /** Constrains the velocity of the intake */
    private final Constraints pidConstraints;
    private static ProfiledPIDController PIDpivotController;

    /** Relitive Encoder */
    private final RelativeEncoder encoder;

    public IntakeSubsystem() {
        // Initiate velocity and acceleration constrainst & PID controller
        pidConstraints = new Constraints(1, 1);
        PIDpivotController = new ProfiledPIDController(
                Constants.INTAKE_CONSTANTS.PID().kP(),
                Constants.INTAKE_CONSTANTS.PID().kI(),
                Constants.INTAKE_CONSTANTS.PID().kD(),
                pidConstraints);

        // configure intake motor
        intakeConfig
                .inverted(false)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(40)
                .closedLoopRampRate(0.001);
        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // configure pivot motor
        pivotConfig
                .inverted(false)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(40)
                .closedLoopRampRate(0.001);

        pivotConfig.alternateEncoder
                .setSparkMaxDataPortConfig()
                .positionConversionFactor(pivotRatio);

        pivotMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = pivotMotor.getAlternateEncoder();
        encoder.setPosition(0); // zero encoder
    }

    /**
     * 
     * = = = = = = = = = = METHODS = = = = = = = = = =
     * 
     */

    /**
     * Set the intake up or down
     * 
     * @param goalUp TRUE: set the intake to the up position, FALSE: set intake to
     *               down/on the ground position
     */
    private void setGoal(boolean goalUp) {
        // Goal rotation of the intake's REAL PIVOT, not the motor
        double goalRotations;
        if (goalUp)
            goalRotations = 0;
        else
            goalRotations = 0.25;

        double PIDoutput = PIDpivotController.calculate(encoder.getPosition(), goalRotations);
        PIDoutput = MathUtil.clamp(PIDoutput, -1, 1); // Clamp the value bc motor can not go > or < 100%

        pivotMotor.set(PIDoutput);
    }

    private void runIntakeMotor() {
        intakeMotor.set(1);
    }

    private void stopIntakeMotor() {
        intakeMotor.set(0);
    }

    /**
     * Start the intake
     * 
     * @return Command that starts the intake
     */
    public Command runIntake() {
        return run(() -> runIntakeMotor());
    }

    /**
     * stop the intake
     * 
     * @return Command that stops the intake
     */
    public Command stopIntake() {
        return run(() -> stopIntakeMotor());
    }

    @Override
    public void periodic() {

    }
}

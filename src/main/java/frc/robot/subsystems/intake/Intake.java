package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.networktables.GenericEntry;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;

/**
 * Subsystem for a rack-and-pinion intake.
 */
public class Intake extends SubsystemBase {

    // TODO: Update these to match the actual CAN IDs
    private static final int PINION_MOTOR_ID = 9;
    private static final int INTAKE_MOTOR_ID = 10;

    /**
     * Drives the rack.
     */
    private static final SparkMax pinionMotor = new SparkMax(PINION_MOTOR_ID, MotorType.kBrushless);
    /**
     * Spins the intake wheels.
     */
    private static final SparkMax intakeMotor = new SparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);

    private final SparkMaxConfig pinionConfig = new SparkMaxConfig();
    private final SparkMaxConfig intakeConfig = new SparkMaxConfig();

    private final RelativeEncoder pinionEncoder;

    // TODO: Measure and update this value!
    private static final double MAX_EXTENSION_ROTATIONS = 3.0;
    private static final double MIN_EXTENSION_ROTATIONS = 0.0;

    // TODO: Measure and update these values!
    private static final double RETRACTED_POSITION = 0.2;
    private static final double EXTENDED_POSITION = 2.8;

    private static final double kP = 0.55;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    private static final double CRUISE_VELOCITY = 120; // RPM
    private static final double MAX_ACCELERATION = 20; // RPM/s
    private static final double ALLOWED_ERROR = 0.5; // rotations

    // When the motor stalls against the hard stop, current exceeds this.
    // Adjust if the intake homes too early (lower) or too late (higher)
    private static final double HOMING_CURRENT_THRESHOLD = 15.0; // amps
    // Speed to retract during homing (negative = retract direction).
    private static final double HOMING_SPEED = -0.1;

    private boolean isHomed = false;
    private double setPoint = 0.0;

    private static final GenericEntry positionEntry = Shuffleboard
            .getTab("Intake")
            .add("Rack position", 0)
            .getEntry();
    private static final GenericEntry setpointEntry = Shuffleboard
            .getTab("Intake")
            .add("Setpoint", 0)
            .getEntry();
    private static final GenericEntry currentEntry = Shuffleboard
            .getTab("Intake")
            .add("Pinion current", 0)
            .getEntry();
    private static final GenericEntry homedEntry = Shuffleboard
            .getTab("Intake")
            .add("Homed", false)
            .getEntry();

    public Intake() {
        intakeConfig
                .inverted(true)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(20)
                .closedLoopRampRate(0.001);
        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        pinionConfig
                .inverted(false) // TODO: Set based on which direction extends
                .idleMode(IdleMode.kBrake) // Peter: I think we want brake to hold the position
                .smartCurrentLimit(40);

        pinionConfig.closedLoop
                .pid(kP, kI, kD);
        pinionConfig.closedLoop.maxMotion
                .cruiseVelocity(CRUISE_VELOCITY)
                .maxAcceleration(MAX_ACCELERATION)
                .allowedProfileError(ALLOWED_ERROR);

        pinionConfig.softLimit
                .forwardSoftLimitEnabled(true)
                .forwardSoftLimit(MAX_EXTENSION_ROTATIONS)
                .reverseSoftLimitEnabled(true)
                .reverseSoftLimit(MIN_EXTENSION_ROTATIONS);

        pinionMotor.configure(pinionConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        pinionEncoder = pinionMotor.getEncoder();
    }

    public Command homeCommand() {
        return run(() -> {
            // Drive slowly toward retracted hard stop
            pinionMotor.set(HOMING_SPEED);
        })
                .until(() -> {
                    // TODO: We might need to sample this?
                    return pinionMotor.getOutputCurrent() > HOMING_CURRENT_THRESHOLD;
                })
                // Might need adjusting. We use a timeout as a safety net.
                .withTimeout(1.5)
                .finallyDo((interrupted) -> {
                    pinionMotor.set(0);
                    if (!interrupted) {
                        // We hit the hard stop; zero the encoder here
                        pinionEncoder.setPosition(0.0);
                        isHomed = true;
                        homedEntry.setBoolean(true);
                    }
                });
    }

    /** Whether the intake has been homed since the last enable. */
    public boolean isHomed() {
        return isHomed;
    }

    /** Extend the intake to the deployed position. */
    public void extend() {
        if (!isHomed) {
            // We should probably throw an exception rather than silently fail
            return;
        }
        setPoint = EXTENDED_POSITION;
        setpointEntry.setDouble(setPoint);
    }

    /** Retract the intake to the stowed position. */
    public void retract() {
        if (!isHomed) {
            return;
        }
        setPoint = RETRACTED_POSITION;
        setpointEntry.setDouble(setPoint);
    }

    /** Run the intake wheels. */
    public void run() {
        intakeMotor.set(0.8);
    }

    /** Stop the intake wheels. */
    public void stop() {
        intakeMotor.set(0);
    }

    /** Get the current position in motor rotations from home. */
    public double getPosition() {
        return pinionEncoder.getPosition();
    }

    private int telemetryCounter = 0;

    @Override
    public void periodic() {
        if (isHomed) {
            pinionMotor.getClosedLoopController().setSetpoint(
                    setPoint, SparkBase.ControlType.kMAXMotionPositionControl);
        }

        if (++telemetryCounter >= 10) {
            positionEntry.setDouble(pinionEncoder.getPosition());
            currentEntry.setDouble(pinionMotor.getOutputCurrent());
            telemetryCounter = 0;
        }
    }
}
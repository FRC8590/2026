package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.GenericEntry;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;

/**
 * Subsystem for a rack-and-pinion intake.
 */
public class Intake extends SubsystemBase {

    // TODO: Update these to match the actual CAN IDs
    private static final int PINION_MOTOR_ID = 10;
    private static final int INTAKE_MOTOR_ID = 15;

    /**
     * Drives the rack.
     */
    private static final SparkMax pinionMotor = new SparkMax(PINION_MOTOR_ID, MotorType.kBrushless);
    /**
     * Spins the intake wheels.
     */
    private static final SparkFlex intakeMotor = new SparkFlex(INTAKE_MOTOR_ID, MotorType.kBrushless);

    private final SparkMaxConfig pinionConfig = new SparkMaxConfig();
    private final SparkMaxConfig intakeConfig = new SparkMaxConfig();

    private final RelativeEncoder pinionEncoder;

    private static final double MAX_EXTENSION_ROTATIONS = 12.4;
    private static final double MIN_EXTENSION_ROTATIONS = 0.2;

    private static final double RETRACTED_POSITION = 1;
    private static final double EXTENDED_POSITION = 12.3;

    private static final double kP = 20.0;
    private static final double kI = 0.0;
    private static final double kD = 0.07;

    private static final double CRUISE_VELOCITY = 7000; // RPM
    private static final double MAX_ACCELERATION = 7000; // RPM/s
    private static final double ALLOWED_ERROR = 2.5; // rotations

    // When the motor stalls against the hard stop, current exceeds this.
    // Adjust if the intake homes too early (lower) or too late (higher)
    private static final double HOMING_CURRENT_THRESHOLD = 80.0; // amps
    // Speed to retract during homing (negative = retract direction).
    private static final double HOMING_SPEED = -1;

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
    private static final GenericEntry pidEntry = Shuffleboard
            .getTab("Intake")
            .add("PID output", 0.00)
            .getEntry();
    private static final GenericEntry hardStopEntry = Shuffleboard
            .getTab("Intake")
            .add("Hard Stopped", false)
            .getEntry();
    private static final GenericEntry intakeAtSetpoint = Shuffleboard
            .getTab("Intake")
            .add("At setpoint", false)
            .getEntry();

    public Intake() {
        intakeConfig
                .inverted(true)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(60);
        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        pinionConfig
                .inverted(true)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(80);

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

    private boolean hardStopped = false;

    public void initialize() {
        SmartDashboard.putData("Intake", Commands.runOnce(this::hardStop).withName("Intake hard stop"));
    }

    private void hardStop() {
        hardStopped = !hardStopped;
        hardStopEntry.setBoolean(hardStopped);
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
        intakeMotor.set(-0.5); // ~6800 RPM :)
    }

    /** Stop the intake wheels. */
    public void stop() {
        intakeMotor.set(0);
    }

    /** Get the current position in motor rotations from home. */
    public double getPosition() {
        return pinionEncoder.getPosition();
    }

    /**
     * @return
     */
    public boolean isAtSetpoint() {
        return pinionMotor.getClosedLoopController().isAtSetpoint();
    }

    private int telemetryCounter = 0;

    @Override
    public void periodic() {
        if (hardStopped) {
            pinionMotor.set(0);
            return;
        }

        if (isHomed) {
            /*
            if (setPoint == EXTENDED_POSITION) {
                pinionMotor.set(0.2); // .5 seconds to extend and 12 rotations for full extension = 1440rpm
            } else {
                pinionMotor.set(-1);
            }*/
            pinionMotor.getClosedLoopController().setSetpoint(
                    setPoint, SparkBase.ControlType.kMAXMotionPositionControl);
        }

        if (++telemetryCounter >= 10) {
            positionEntry.setDouble(pinionEncoder.getPosition());
            currentEntry.setDouble(pinionMotor.getOutputCurrent());
            pidEntry.setDouble(pinionMotor.getAppliedOutput() * 12);
            intakeAtSetpoint.setBoolean(isAtSetpoint());
            telemetryCounter = 0;
        }
    }
}
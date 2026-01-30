package frc.robot.subsystems.swervedrive;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

public class IntakeSubsystem extends SubsystemBase
{
    private final SparkMax pivotMotor = new SparkMax(Constants.INTAKE_CONSTANTS.pivotMotorID(), MotorType.kBrushless);
    private final SparkMax intakeMotor = new SparkMax(Constants.INTAKE_CONSTANTS.intakeMotorID(), MotorType.kBrushless);

    private final SparkMaxConfig intakeConfig = new SparkMaxConfig();

    public IntakeSubsystem ()
    {
        intakeConfig
            .inverted(false)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(40)
            .closedLoopRampRate(0.001);
        pivotMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void runPivotMotor ()
    {
        pivotMotor.set(1);
    }

    private void stopPivotMotor ()
    {
        pivotMotor.set(0);
    }

    private void runIntakeMotor ()
    {
        intakeMotor.set(1);
    }

    private void stopIntakeMotor ()
    {
        intakeMotor.set(0);
    }

    /**
     * (eventually will) Puts the intake down
     * @return Command that puts the intake in the down position
     */
    public Command runPivot ()
    {
        return run(() -> runPivotMotor());
    }

    /**
     * (eventualy will) put the intake up
     * @return Command that puts the intake in the up position
     */
    public Command stopPivot ()
    {
        return run(() -> stopPivotMotor());
    }

    /**
     * Start the intake
     * @return Command that starts the intake
     */
    public Command runIntake ()
    {
        return run(() -> runIntakeMotor());
    }

    /**
     * stop the intake
     * @return Command that stops the intake
     */
    public Command stopIntake ()
    {
        return run(() -> stopIntakeMotor());
    }
}

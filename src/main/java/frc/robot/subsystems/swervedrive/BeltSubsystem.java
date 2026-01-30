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

public class BeltSubsystem extends SubsystemBase
{
    private final SparkMax beltMotor = new SparkMax(Constants.BELT_CONSTANTS.beltMotorID(), MotorType.kBrushless);

    private final SparkMaxConfig beltMotorConfig = new SparkMaxConfig();

    public BeltSubsystem ()
    {
        beltMotorConfig
            .inverted(false)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(40)
            .closedLoopRampRate(0.001);
        
        beltMotor.configure(beltMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void runMotor ()
    {
        beltMotor.set(1);
    }

    private void stopMotor ()
    {
        beltMotor.set(0);
    }

    /**
     * Runs the belt in the fuel container
     * @return Command that sets the belt motor to max speed
     */
    public Command runBelt ()
    {
        return run(() -> runMotor());
    }

    /**
     * stop the belt in the fuel container
     * @return Command that stops the belt motor
     */
    public Command stopBelt ()
    {
        return run(() -> stopMotor());
    }
}

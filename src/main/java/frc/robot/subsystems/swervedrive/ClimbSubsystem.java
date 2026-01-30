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

public class ClimbSubsystem extends SubsystemBase
{
    /**
     * Climb will deffinetly be more complicated,
     * but this is a start 
     * - Riley
     */
    private final SparkMax climbMotor = new SparkMax(Constants.CLIMB_CONSTANTS.climbMotorID(), MotorType.kBrushless);

    private final SparkMaxConfig climbMotorConfig = new SparkMaxConfig();

    public ClimbSubsystem ()
    {
        climbMotorConfig
            .inverted(false)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(40)
            .closedLoopRampRate(0.001);
        
        climbMotor.configure(climbMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void startClimbMotor ()
    {
        climbMotor.set(1);
    }

    private void stopClimbMotor ()
    {
        climbMotor.set(0);
    }

    /**
     * Start the ladder climb
     * @return Command that starts the ladder climb
     */
    public Command startClimb ()
    {
        return run(() -> startClimbMotor());
    }

    /**
     * stop the ladder climb
     * @return Command that stops the ladder climb
     */
    public Command stopClimb ()
    {
        return run(() -> stopClimbMotor());
    }
}

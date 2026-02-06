package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

public class Shooter extends SubsystemBase
{
    private final SparkMax frontMotor = new SparkMax(Constants.SHOOTER_CONSTANTS.frontMotorID(), MotorType.kBrushless);
    private final SparkMax backMotor = new SparkMax(Constants.SHOOTER_CONSTANTS.backMotorID(), MotorType.kBrushless);

    private final SparkMaxConfig shooterConfig = new SparkMaxConfig();

    public Shooter ()
    {
        shooterConfig
            .inverted(false)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(40)
            .closedLoopRampRate(0.001);
        frontMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        shooterConfig.inverted(true);
        backMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    private void runMotors ()
    {
        frontMotor.set(1);
        backMotor.set(1);
    }

    private void stopMotors ()
    {
        frontMotor.set(0);
        backMotor.set(0);
    }

    /**
     * runs both shooter motors at full speed
     * @return Command that sets both motors to full speed
     */
    public Command runShooter ()
    {
        return run(() -> runMotors());
    }

    /**
     * stops both shooter motors
     * @return Command that stops both motors
     */
    public Command stopShooter ()
    {
        return run(() -> stopMotors());
    }
}

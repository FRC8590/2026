package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

public class Shooter extends SubsystemBase {
    private final SparkFlex frontMotor = new SparkFlex(Constants.SHOOTER_CONSTANTS.frontMotorID(), MotorType.kBrushless);
    private final SparkFlex backMotor = new SparkFlex(Constants.SHOOTER_CONSTANTS.backMotorID(), MotorType.kBrushless);

    private final SparkFlexConfig shooterConfig = new SparkFlexConfig();

    public Shooter() {
        shooterConfig
                .inverted(true)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(40)
                .closedLoopRampRate(0.001);
        frontMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        shooterConfig.inverted(false);
        backMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    private void runMotors(double speed) {
        frontMotor.set(-speed);
        backMotor.set(-speed);
    }

    private void stopMotors() {
        frontMotor.set(0);
        backMotor.set(0);
    }

    /**
     * runs both shooter motors at full speed
     * 
     * @param speed number from -1 to 1 for the motors to run at
     * @return Command that sets both motors to full speed
     */
    public Command runShooter(double speed) {
        return run(() -> runMotors(speed));
    }

    /**
     * stops both shooter motors
     * 
     * @return Command that stops both motors
     */
    public Command stopShooter() {
        return run(() -> stopMotors());
    }
}

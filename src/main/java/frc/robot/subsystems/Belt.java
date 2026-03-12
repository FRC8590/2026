package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

public class Belt extends SubsystemBase {
    private final SparkMax beltMotor = new SparkMax(Constants.BELT_CONSTANTS.beltMotorID(), MotorType.kBrushless);
    private final SparkFlex indexMotor = new SparkFlex(Constants.BELT_CONSTANTS.indexMotorID(), MotorType.kBrushless);


    private final SparkMaxConfig beltMotorConfig = new SparkMaxConfig();
    private final SparkFlexConfig indexMotorConfig = new SparkFlexConfig();

    public Belt() {
        beltMotorConfig
                .inverted(true)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(40)
                .closedLoopRampRate(0.001);

        indexMotorConfig
                .inverted(false)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(40)
                .closedLoopRampRate(0.001);
        beltMotor.configure(beltMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        indexMotor.configure(indexMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void runMotor() {
        System.out.println("start moter");
        beltMotor.set(1);
        indexMotor.set(1);
    }

    private void runMotorReversed ()
    {
        beltMotor.set(-.8);
        indexMotor.set(-.8);
    }

    private void stopMotor() {
        System.out.println("stop motor");
        beltMotor.set(0);
        indexMotor.set(0);
    }

    /**
     * Runs the belt in the fuel container
     * 
     * @return Command that sets the belt motor to max speed
     */
    public Command runBelt() {
        return run(() -> runMotor());
    }

    /**
     * stop the belt in the fuel container
     * 
     * @return Command that stops the belt motor
     */
    public Command stopBelt() {
        return run(() -> stopMotor());
    }

    public Command runBeltReversed ()
    {
        return run(()-> runMotorReversed());
    }
}

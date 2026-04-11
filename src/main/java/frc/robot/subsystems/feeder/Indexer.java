package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.ResetMode;

import com.revrobotics.PersistMode;

/* Subsystem for the indexer wheels (belt not included). */
public class Indexer extends SubsystemBase {
    private static final int INDEX_MOTOR_ID = 14;

    private static final SparkFlex indexMotor = new SparkFlex(INDEX_MOTOR_ID, MotorType.kBrushless);

    private final SparkFlexConfig indexMotorConfig = new SparkFlexConfig();

    private static final GenericEntry indexerEntry = Shuffleboard.getTab("Shooter")
            .add("Index motor RPM", 0)
            .getEntry();

    public Indexer() {
        indexMotorConfig
                .inverted(false)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(60)
                .closedLoopRampRate(0.001);

        indexMotor.configure(indexMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /* Run the indexer. */
    public void run() {
        indexMotor.set(1);
    }

    /* Stop the indexer. */
    public void stop() {
        indexMotor.set(0);
    }

    /* Run the indexer in reverse. */
    public void runReversed() {
        indexMotor.set(-1);
    }

    public double getSpeed() {
        return indexMotor.get();
    }

    private int telemetryCounter = 0;

    @Override
    public void periodic() {
        if (++telemetryCounter >= 30) {
            indexerEntry.setDouble(indexMotor.getEncoder().getVelocity() / 6);
            telemetryCounter = 0;
        }
    }
}

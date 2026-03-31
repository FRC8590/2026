package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;

import com.revrobotics.PersistMode;

/* Subsystem for the belt. */
public class Belt extends SubsystemBase {
    private static final int beltMotorID = 11;

    private static final SparkMax beltMotor = new SparkMax(beltMotorID, MotorType.kBrushless);

    private final SparkMaxConfig beltMotorConfig = new SparkMaxConfig();

    private static final GenericEntry beltEntry = Shuffleboard.getTab("Shooter")
            .add("Belt motor RPM", 0)
            .getEntry();

    public Belt() {
        beltMotorConfig
                .inverted(true)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(40)
                .closedLoopRampRate(0.001);

        beltMotor.configure(beltMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /* Run the belt. */
    public void run() {
        beltMotor.set(1);
    }

    /* Stop the belt. */
    public void stop() {
        beltMotor.set(0);
    }

    /* Run the belt in reverse. */
    public void runReversed() {
        beltMotor.set(-1);
    }

    public double getSpeed() {
        return beltMotor.get();
    }

    private int telemetryCounter = 0;

    @Override
    public void periodic() {
        if (++telemetryCounter >= 30) {
            beltEntry.setDouble(beltMotor.getEncoder().getVelocity() / 6);
            telemetryCounter = 0;
        }
    }
}

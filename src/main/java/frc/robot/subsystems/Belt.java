package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;

import com.revrobotics.PersistMode;

public class Belt extends SubsystemBase {
    private final int beltMotorID = 11;

    private final SparkMax beltMotor = new SparkMax(beltMotorID, MotorType.kBrushless);

    private final SparkMaxConfig beltMotorConfig = new SparkMaxConfig();

    private final GenericEntry beltEntry = Shuffleboard.getTab("Shooter")
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

    public void run() {
        beltMotor.set(1);
    }

    public void stop() {
        beltMotor.set(0);
    }

    public void runReversed() {
        beltMotor.set(-1);
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

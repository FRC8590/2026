package frc.robot.subsystems;

import frc.robot.Systems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
    private final int beltMotorID = 11;
    private final int indexMotorID = 14;

    private final SparkMax beltMotor = new SparkMax(beltMotorID, MotorType.kBrushless);
    private final SparkFlex indexMotor = new SparkFlex(indexMotorID, MotorType.kBrushless);

    private final SparkMaxConfig beltMotorConfig = new SparkMaxConfig();
    private final SparkFlexConfig indexMotorConfig = new SparkFlexConfig();

    private final GenericEntry beltEntry = Shuffleboard.getTab("Shooter")
            .add("Belt motor RPM", 0)
            .getEntry();
    private final GenericEntry indexerEntry = Shuffleboard.getTab("Shooter")
            .add("Index motor RPM", 0)
            .getEntry();

    private final Shooter shooterSystem;

    public Belt(Shooter shooter) {
        shooterSystem = shooter;

        beltMotorConfig
                .inverted(true)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(40)
                .closedLoopRampRate(0.001);

        indexMotorConfig
                .inverted(false)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(60)
                .closedLoopRampRate(0.001);

        beltMotor.configure(beltMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        indexMotor.configure(indexMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void setIndexMotorSpeed(double speed) {
        if (Systems.isSystemEnabled(Systems.enableIndexer)) {
            indexMotor.set(speed);
        }
    }

    private void setBeltMotorSpeed(double speed) {
        if (Systems.isSystemEnabled(Systems.enableBelt)) {

            beltMotor.set(speed);
        }
    }

    private void runBelt() {
        setBeltMotorSpeed(1);
    }

    private void runIndexer() {
        setIndexMotorSpeed(0.5);
    }

    private void stopBelt() {
        setBeltMotorSpeed(0);
    }

    private void stopIndexer() {
        setIndexMotorSpeed(0);
    }

    private void runBeltReversed() {
        setBeltMotorSpeed(-1);
    }

    private void runIndexerReversed() {
        setIndexMotorSpeed(-0.5);
    }

    public void runBeltAndIndexer() {
        if (shooterSystem.atRPM()) {
            runBelt();
            runIndexer();
        }
    }

    public void stopBeltAndIndexer() {
        stopBelt();
        stopIndexer();
    }

    /**
     * Runs the belt in the fuel container
     * 
     * @return Command that sets the belt motor to max speed
     */
    public Command beltRun() {
        return run(() -> runBelt());
    }

    /**
     * stop the belt in the fuel container
     * 
     * @return Command that stops the belt motor
     */
    public Command beltStop() {
        return run(() -> stopBelt());
    }

    public Command beltRunReversed() {
        return run(() -> runBeltReversed());
    }

    public Command indexerRun() {
        return run(() -> runIndexer());
    }

    public Command indexerStop() {
        return run(() -> stopIndexer());
    }

    public Command indexerRunReversed() {
        return run(() -> runIndexerReversed());
    }

    public Command beltAndIndexerRun() {
        return run(() -> runBeltAndIndexer());
    }

    public Command beltAndIndexerStop() {
        return run(() -> stopBeltAndIndexer());
    }

    private int telemetryCounter = 0;

    @Override
    public void periodic() {
        if (++telemetryCounter >= 50) {
            beltEntry.setDouble(beltMotor.getEncoder().getVelocity() / 6);
            indexerEntry.setDouble(indexMotor.getEncoder().getVelocity() / 6);
            telemetryCounter = 0;
        }
    }
}

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Systems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
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

    private GenericEntry beltEntry;
    private GenericEntry indexerEntry;

    public Belt() {
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

        // Shuffleboard setup
        ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");

        ShuffleboardLayout beltLayout = shooterTab.getLayout("Indexer", BuiltInLayouts.kList).withSize(2, 2)
                .withPosition(2, 0);
        SimpleWidget beltWidget = beltLayout.add("Belt motor RPM", 0);
        SimpleWidget indexerWidget = beltLayout.add("Indexer motor RPM", 0);

        indexerEntry = indexerWidget.getEntry();
        beltEntry = beltWidget.getEntry();
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
        if (Constants.shooter.atRPM()) {
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

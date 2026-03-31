package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.ResetMode;

import java.util.Map;

import com.revrobotics.PersistMode;

/* Subsystem for the shooter. */
public class Shooter extends SubsystemBase {

    private static final int frontMotorID = 12;
    private static final int backMotorID = 13;

    private static final SparkFlex frontMotor = new SparkFlex(frontMotorID, MotorType.kBrushless);
    private static final SparkFlex backMotor = new SparkFlex(backMotorID, MotorType.kBrushless);

    private final SparkFlexConfig shooterConfig = new SparkFlexConfig();

    public static final double SHOOTER_MAX_RPM = 6784;

    private double goalRPM = 0;

    private final double p = 0.0001;
    private final double i = 0;
    private final double d = 0;
    private final double kA = 0;
    private final double kV = 0.0019;
    // Cruise velocity
    private double cruiseVelocity = 6700;
    // Max acceleration
    private double maxAcceleration = 4000; // TODO: increase

    private static final GenericEntry targRPMEntry = Shuffleboard
            .getTab("Shooter")
            .add("Target RPM", 0)
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of("min", 0, "max", SHOOTER_MAX_RPM))
            .getEntry();

    private static final GenericEntry currRPMFrontEntry = Shuffleboard
            .getTab("Shooter")
            .add("Front motor RPM", 0)
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of("Min", 0, "Max", SHOOTER_MAX_RPM))
            .getEntry();

    private static final GenericEntry currRPMBackEntry = Shuffleboard
            .getTab("Shooter")
            .add("Back motor RPM", 0)
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of("Min", 0, "Max", SHOOTER_MAX_RPM))
            .getEntry();

    public Shooter() {
        shooterConfig
                .inverted(false)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(60)
                .closedLoopRampRate(0.001); // TODO: look at this
        shooterConfig.closedLoop
                .pid(p, i, d);
        shooterConfig.closedLoop.feedForward
                .kA(kA)
                .kV(kV);
        shooterConfig.closedLoop.maxMotion
                .cruiseVelocity(cruiseVelocity)
                .maxAcceleration(maxAcceleration);

        frontMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        shooterConfig.inverted(true);
        backMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public double getGoalRPM() {
        return goalRPM;
    }

    /*
     * Set the goal RPM on the shooter.
     * 
     * Note that this will not immediately move the motor.
     * Instead, the motor will begin speeding up on the next periodic() call.
     * 
     * Use atRPM() to detect when the motor is at the desired RPM.
     */
    public void setGoalRPM(double rpm) {
        targRPMEntry.setDouble(rpm);
        goalRPM = rpm;
    }

    // Peter: This was written by Claude based on some rough estimates.
    // We should get rid of this once we have an actual regression model,
    // but until we get that chance, this is about as good as we can do.
    /**
     * Calculates required shooter RPM for a given horizontal distance to the hub,
     * using projectile motion physics.
     *
     * Assumes:
     * Launch angle: 69 degrees
     * Shooter height: ~0.61m (24 inches)
     * Hub height: 1.83m (72 inches)
     * Wheel diameter: 0.0889m (3.5 inches)
     * Wheel slip factor: 0.9 (tune this first if readings are off)
     *
     * @param distanceMeters Horizontal distance from shooter to hub in meters
     * @return Required RPM, clamped to SHOOTER_MAX_RPM
     */
    public static double distanceToRPM(double distanceMeters) {
        final double g = 9.81;
        final double angleRad = Math.toRadians(69);
        final double shooterHeight = 0.0; // sim launches from ground level
        final double hubHeight = 1.83;
        final double deltaY = hubHeight - shooterHeight;
        final double MIN_DISTANCE = 0.75;
        final double MAX_DISTANCE = 6.0;

        if (distanceMeters < MIN_DISTANCE || distanceMeters > MAX_DISTANCE) {
            System.err.println("distanceToRPM: outside viable range (" + distanceMeters + "m)");
            return 0;
        }

        double cosA = Math.cos(angleRad);
        double tanA = Math.tan(angleRad);
        double denominator = distanceMeters * tanA - deltaY;

        if (denominator <= 0) {
            System.err.println("distanceToRPM: degenerate denominator at " + distanceMeters + "m");
            return 0;
        }

        double v0Squared = (g * distanceMeters * distanceMeters)
                / (2 * cosA * cosA * denominator);
        double v0 = Math.sqrt(v0Squared);

        // Use the sim's exact RPM<->velocity model: 6000 RPM = 20 m/s
        double rpm = v0 * 6000.0 / 20.0;

        // We arbitrarily add 100 to account for some slight undershooting
        return Math.min(rpm + 100, SHOOTER_MAX_RPM);
    }

    /**
     * If the average RPM of the front and back shooter motors is at or above the
     * goal RPM
     * 
     * @return true if average RPM is at or above goal RPM
     */
    public boolean atRPM() {
        return (frontMotor.getEncoder().getVelocity() + backMotor.getEncoder().getVelocity()) / 2 >= goalRPM - 100;
    }

    private int telemetryCounter = 0;

    @Override
    public void periodic() {
        frontMotor.getClosedLoopController().setSetpoint(goalRPM, SparkBase.ControlType.kMAXMotionVelocityControl);
        backMotor.getClosedLoopController().setSetpoint(goalRPM, SparkBase.ControlType.kMAXMotionVelocityControl);

        if (++telemetryCounter >= 20) {
            targRPMEntry.setDouble(goalRPM);
            currRPMBackEntry.setDouble(backMotor.getEncoder().getVelocity() / 6);
            currRPMFrontEntry.setDouble(frontMotor.getEncoder().getVelocity() / 6);
            telemetryCounter = 0;
        }
    }
}

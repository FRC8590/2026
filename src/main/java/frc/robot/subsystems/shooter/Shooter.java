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

    private final int frontMotorID = 12;
    private final int backMotorID = 13;

    private final SparkFlex frontMotor = new SparkFlex(frontMotorID, MotorType.kBrushless);
    private final SparkFlex backMotor = new SparkFlex(backMotorID, MotorType.kBrushless);

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

    private final GenericEntry targRPMEntry = Shuffleboard
            .getTab("Shooter")
            .add("Target RPM", 0)
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of("min", 0, "max", SHOOTER_MAX_RPM))
            .getEntry();

    private final GenericEntry currRPMFrontEntry = Shuffleboard
            .getTab("Shooter")
            .add("Front motor RPM", 0)
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of("Min", 0, "Max", SHOOTER_MAX_RPM))
            .getEntry();

    private final GenericEntry currRPMBackEntry = Shuffleboard
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
     * Launch angle: 29 degrees
     * Shooter height: ~0.61m (24 inches)
     * Hub height: 1.83m (72 inches)
     * Wheel diameter: 0.0889m (3.5 inches)
     * Wheel slip factor: 0.9 (tune this first if readings are off)
     *
     * @param distanceMeters Horizontal distance from shooter to hub in meters
     * @return Required RPM, clamped to SHOOTER_MAX_RPM
     */
    public static double distanceToRPM(double distanceMeters) {
        // Note that air resistance is ignored; this might not
        // work at longer ranges, but that shouldn't matter.

        final double g = 9.81;
        final double angleRad = Math.toRadians(29);
        final double shooterHeight = 0.61; // meters
        final double hubHeight = 1.83; // meters
        final double deltaY = hubHeight - shooterHeight; // 1.22m
        final double wheelDiameter = 0.0889; // meters

        /*
         * If shots are consistently falling short, increase the slip factor
         * (wheels need to spin faster to compensate for more slip). If
         * overshooting, decrease it. Each 0.05 change moves the output
         * roughly 5-6%.
         */
        final double slipFactor = 0.9;

        double cosA = Math.cos(angleRad);
        double tanA = Math.tan(angleRad);

        double denominator = distanceMeters * tanA - deltaY;

        // Guard against impossible shots (too close, or angle can't reach target)
        if (denominator <= 0) {
            System.err.println("distanceToRPM: target unreachable at distance " + distanceMeters + "m");
            return 0;
        }

        double v0Squared = (g * distanceMeters * distanceMeters)
                / (2 * cosA * cosA * denominator);
        double v0 = Math.sqrt(v0Squared);

        double wheelCircumference = Math.PI * wheelDiameter;
        double rpm = (v0 / slipFactor) / wheelCircumference * 60;

        return Math.min(rpm, SHOOTER_MAX_RPM);
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

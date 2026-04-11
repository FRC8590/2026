package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.woodsonrobotics.math.LiveRegression;
import lib.woodsonrobotics.telemetry.notify.DriveNotifier;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.ClosedLoopSlot;
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

    private static final int FRONT_MOTOR_ID = 12;
    private static final int BACK_MOTOR_ID = 13;

    public static final double SHOOTER_MAX_RPM = 6784;

    // Model (distance only, not scoring): y = 426.11258x + 460.08048
    public static final double RPM_PER_MPS = 426.11258;
    public static final double RPM_OFFSET = 460.08048;

    private static final SparkFlex frontMotor = new SparkFlex(FRONT_MOTOR_ID, MotorType.kBrushless);
    private static final SparkFlex backMotor = new SparkFlex(BACK_MOTOR_ID, MotorType.kBrushless);

    private final SparkFlexConfig shooterConfig = new SparkFlexConfig();

    private static final LiveRegression liveRegression = new LiveRegression();

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
                .pid(p, i, d)
                .allowedClosedLoopError(0.3, ClosedLoopSlot.kSlot0); // TODO: find good error tolerance
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

    // TODO: Peter: Supposedly, calling this in the constructor can result in
    // the key not showing up, because SmartDashboard isn't ready yet at that
    // stage. If that turns out to be incorrect, then let's remove this.
    public void initialize() {
        SmartDashboard.putData("Shooting", Commands.runOnce(this::markScore).withName("Scored"));
    }

    public double registeredRPM = -1;
    public double registeredDistance = -1;

    private void markScore() {
        if (registeredRPM == -1 || registeredDistance == -1) {
            DriveNotifier.operatorError("No point is set");
            return;
        }
        liveRegression.addPoint(registeredDistance, registeredRPM);
        System.out.println("Scored, new live regression: " + liveRegression);
        if (liveRegression.isReady()) {
            DriveNotifier.inform("Live regression is ready!");
        }
    }

    public void registerLiveAdjustedRpmAndDistance(double rpm, double distance) {
        registeredRPM = rpm;
        registeredDistance = distance;
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

    /**
     * Calculates required shooter RPM for a given horizontal distance.
     */
    public static double distanceToRPM(double distanceMeters) {
        if (liveRegression.isReady()) {
            return liveRegression.predict(distanceMeters);
        }
        double rpm = (RPM_PER_MPS * distanceMeters) + RPM_OFFSET;
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

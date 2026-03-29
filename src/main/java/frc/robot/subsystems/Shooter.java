package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Systems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.Command;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.ResetMode;

import static edu.wpi.first.units.Units.Inches;

import java.util.Map;

import com.revrobotics.PersistMode;

public class Shooter extends SubsystemBase {
    private final SparkFlex frontMotor = new SparkFlex(Constants.SHOOTER_CONSTANTS.frontMotorID(),
            MotorType.kBrushless);
    private final SparkFlex backMotor = new SparkFlex(Constants.SHOOTER_CONSTANTS.backMotorID(), MotorType.kBrushless);

    private final SparkFlexConfig shooterConfig = new SparkFlexConfig();

    public static final double SHOOTER_MAX_RPM = 6784;

    /** cV: cruiseVelocity. mA: maxAcceleration */
    private double goalRPM = 0, p, i, d, kA, kV, cV, mA;

    private GenericEntry targRPMEntry;
    private GenericEntry currRPMFrontEntry;
    private GenericEntry currRPMBackEntry;
    private GenericEntry setRPMEntry;

    public Shooter() {
        p = 0.0001;
        i = 0;
        d = 0;
        kA = 0;
        kV = 0.0019;
        cV = 6700;
        mA = 4000; // todo increase

        shooterConfig // configure motors
                .inverted(false)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(60)
                .closedLoopRampRate(0.001); // todo look at this
        shooterConfig.closedLoop // configure PID
                .pid(p, i, d);
        shooterConfig.closedLoop.feedForward
                .kA(kA)
                .kV(kV);
        shooterConfig.closedLoop.maxMotion // configure maxMotion
                .cruiseVelocity(cV)
                .maxAcceleration(mA);

        frontMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        shooterConfig.inverted(true);
        backMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // Shuffleboard setup
        ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");

        ShuffleboardLayout shooterLayout = shooterTab.getLayout("Shooter", BuiltInLayouts.kGrid).withSize(2, 2)
                .withPosition(4, 0);
        SimpleWidget currRPMFrontWidget = shooterLayout.add("Front motor RPM", 0).withWidget(BuiltInWidgets.kDial)
                .withProperties(Map.of("Min", 0, "Max", SHOOTER_MAX_RPM)).withPosition(0, 1);
        SimpleWidget currRPMBackWidget = shooterLayout.add("Back motor RPM", 0).withWidget(BuiltInWidgets.kDial)
                .withProperties(Map.of("Min", 0, "Max", SHOOTER_MAX_RPM)).withPosition(1, 1);
        SimpleWidget targRPMWidget = shooterLayout.add("Target RPM", 0).withWidget(BuiltInWidgets.kDial)
                .withProperties(Map.of("min", 0, "max", SHOOTER_MAX_RPM)).withPosition(0, 0);

        currRPMFrontEntry = currRPMFrontWidget.getEntry();
        currRPMBackEntry = currRPMBackWidget.getEntry();
        targRPMEntry = targRPMWidget.getEntry();

        setRPMEntry = Shuffleboard
                .getTab("Shooter")
                .add("Stable shoot RPM", 2000)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .getEntry();
    }

    private void setGoalRPM(double rpm) {
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

    /**
     * Change the goal RPM of the shooter. If rpm is greater than 6784, goal RPM is
     * not changed.
     * 
     * @param rpm rotations per minute you want the shooter to run at
     * @return command that sets the goal RPM
     */
    public Command shooterSetGoalRPM(double rpm) {
        if (rpm > SHOOTER_MAX_RPM)
            return run(() -> setGoalRPM(goalRPM));
        return run(() -> {
            if (Robot.isSimulation()) {
                // TODO: Peter: Tune this based on the regression model
                Constants.vision.simulateShoot(8, 29);
            }
            setGoalRPM(rpm);
        });
    }

    public Command shooterSetStableGoalRPM() {
        return run(() -> {
            setGoalRPM(setRPMEntry.getInteger(2000));
        });
    }

    public Command shooterSetRPMFromVision() {
        return run(() -> {
            int primaryId = Vision.getHubAprilTag();
            var result = Constants.vision.getBestSingleTagPoseEstimate(primaryId);
            if (!result.isPresent()) {
                // Nothing seen -- hope for the best!
                return;
            }

            /*
             * The model assumes the distance from getMeasureX() is
             * horizontal distance to the tag, which is generally
             * true, but not exact, depending on where the tag is
             * positioned relative to the hub center. If shots are
             * consistently off by a fixed amount at all distances,
             * we can correct it with a small offset constant.
             */
            double distanceMeters = result.get().getMeasureX().in(Units.Meters);
            double rpm = distanceToRPM(distanceMeters);
            System.out.println("Distance: " + distanceMeters + ". RPM: " + rpm);
            setGoalRPM(rpm);
        });
    }

    /**
     * Sets the goal RPM to zero
     * 
     * @return command that sets the goal RPM to 0
     */
    public Command shooterStop() {
        return run(() -> setGoalRPM(0));
    }

    private int telemetryCounter = 0;

    /**
     * This method will be called once per scheduler run
     * Put values you want to moniter here
     */
    @Override
    public void periodic() {
        if (Systems.isSystemEnabled(Systems.enableShooter)) {
            frontMotor.getClosedLoopController().setSetpoint(goalRPM, SparkBase.ControlType.kMAXMotionVelocityControl);
            backMotor.getClosedLoopController().setSetpoint(goalRPM, SparkBase.ControlType.kMAXMotionVelocityControl);
        }

        if (++telemetryCounter >= 20) {
            targRPMEntry.setDouble(goalRPM);
            currRPMBackEntry.setDouble(backMotor.getEncoder().getVelocity() / 6);
            currRPMFrontEntry.setDouble(frontMotor.getEncoder().getVelocity() / 6);
            telemetryCounter = 0;
        }
    }
}

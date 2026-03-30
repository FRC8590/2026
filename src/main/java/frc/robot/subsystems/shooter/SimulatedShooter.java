package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.ironmaple.utils.FieldMirroringUtils;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import frc.robot.Robot;

public class SimulatedShooter extends Shooter {
    private StructArrayPublisher<Pose3d> fuelPosePublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("Fuel poses", Pose3d.struct)
            .publish();

    public void setGoalRPM(double rpm) {
        super.setGoalRPM(rpm);
        if (rpm > 0) {
            Robot.getInstance().m_robotContainer.drive.ifEnabled(swerve -> {
                var robotSimulationWorldPose = swerve.getPose();
                var chassisSpeedsFieldRelative = swerve.getFieldVelocity();
                RebuiltFuelOnFly fuelOnFly = new RebuiltFuelOnFly(
                        // Specify the position of the chassis when the note is launched
                        robotSimulationWorldPose.getTranslation(),
                        // Specify the translation of the shooter from the robot center (in the
                        // shooter’s reference frame)
                        new Translation2d(0.2, 0),
                        // Specify the field-relative speed of the chassis, adding it to the initial
                        // velocity of the projectile
                        chassisSpeedsFieldRelative,
                        // The shooter facing direction is the same as the robot’s facing direction
                        robotSimulationWorldPose.getRotation(),
                        // Add the shooter’s rotation
                        // + new Rotation2d(90),
                        // Initial height of the flying fuel
                        Inches.of(16.78),
                        // The launch speed is proportional to the RPM; assumed to be 16 meters/second
                        // at 6000 RPM
                        MetersPerSecond.of(rpm / 6000 * 20),
                        // The angle at which the note is launched
                        Radians.of(Math.toRadians(69)));

                fuelOnFly
                        // Set the target center to the Rebbuilt Hub of the current alliance
                        .withTargetPosition(() -> FieldMirroringUtils
                                .toCurrentAllianceTranslation(new Translation3d(0.25, 5.56, 2.3)))
                        // Set the tolerance: x: ±0.5m, y: ±1.2m, z: ±0.3m (this is the size of the
                        // speaker's "mouth")
                        .withTargetTolerance(new Translation3d(0.5, 1.2, 0.3))
                        // Set a callback to run when the fuel hits the target
                        .withHitTargetCallBack(() -> System.out.println("Hit hub, +1 point!"));

                fuelOnFly
                        // Configure the fuel projectile to be "on the field" upon touching the
                        // ground
                        .enableBecomesGamePieceOnFieldAfterTouchGround();

                // Add the projectile to the simulated arena
                SimulatedArena.getInstance().addGamePieceProjectile(fuelOnFly);
            });

        }
    }

    @Override
    public void periodic() {
        super.periodic();
        Pose3d[] fuelPoses = SimulatedArena.getInstance()
                .getGamePiecesArrayByType("Fuel");
        fuelPosePublisher.accept(fuelPoses);
    }

    @Override
    public boolean atRPM() {
        super.atRPM();
        return getGoalRPM() > 0;
    }
}

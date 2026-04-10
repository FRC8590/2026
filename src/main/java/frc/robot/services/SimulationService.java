package frc.robot.services;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import java.util.concurrent.ThreadLocalRandom;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.ironmaple.utils.FieldMirroringUtils;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.feeder.Belt;
import frc.robot.subsystems.feeder.Indexer;
import frc.robot.subsystems.intake.SimulatedIntake;
import frc.robot.subsystems.shooter.Shooter;
import lib.woodsonrobotics.SystemWrapper;

public class SimulationService {
    private final SystemWrapper<SimulatedIntake> intakeSystem;
    private final SystemWrapper<Shooter> shooterSystem;
    private final SystemWrapper<? extends Swerve> driveSystem;
    private final SystemWrapper<Belt> beltSystem;
    private final SystemWrapper<Indexer> indexerSystem;
    private int simulatedScore;
    private final GenericEntry simulatedScoreEntry = Shuffleboard
            .getTab("Console")
            .add("Simulated Score", 0)
            .getEntry();

    private final StructArrayPublisher<Pose3d> fuelPosePublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("Fuel poses", Pose3d.struct)
            .publish();

    public SimulationService(SystemWrapper<SimulatedIntake> intake, SystemWrapper<Shooter> shooter,
            SystemWrapper<? extends Swerve> drive, SystemWrapper<Belt> belt, SystemWrapper<Indexer> indexer) {
        intakeSystem = intake;
        shooterSystem = shooter;
        driveSystem = drive;
        beltSystem = belt;
        indexerSystem = indexer;
    }

    private void addFuelProjectile(Swerve swerve, double rpm, double yValue) {
        ThreadLocalRandom random = ThreadLocalRandom.current();
        var robotSimulationWorldPose = swerve.getPose();
        var chassisSpeedsFieldRelative = swerve.getFieldVelocity();
        double speed = rpm > Shooter.RPM_OFFSET
                ? Math.sqrt((rpm - Shooter.RPM_OFFSET) * 9.81
                        / (Shooter.RPM_PER_MPS * Math.sin(Math.toRadians(138))))
                : 0.0;

        // This is taken from the MapleSim docs
        RebuiltFuelOnFly fuelOnFly = new RebuiltFuelOnFly(
                // Specify the position of the chassis when the note is launched
                robotSimulationWorldPose.getTranslation(),
                // Specify the translation of the shooter from the robot center (in the
                // shooter’s reference frame)
                new Translation2d(0.2, yValue),
                // Specify the field-relative speed of the chassis, adding it to the initial
                // velocity of the projectile
                chassisSpeedsFieldRelative,
                // The shooter facing direction is the same as the robot’s facing direction
                robotSimulationWorldPose.getRotation(),
                // Add the shooter’s rotation
                // + new Rotation2d(90),
                // Initial height of the flying fuel
                Inches.of(16.78),
                // The launch speed is proportional to the RPM
                MetersPerSecond.of(speed),
                // The angle at which the note is launched
                Radians.of(Math.toRadians(69)));

        fuelOnFly
                // Set the target center to the Rebuilt Hub of the current alliance
                .withTargetPosition(() -> FieldMirroringUtils
                        .toCurrentAllianceTranslation(new Translation3d(0.25, 5.56, 2.3)))
                // Set the tolerance: x: ±0.5m, y: ±1.2m, z: ±0.3m (this is the size of the
                // hub's "mouth")
                .withTargetTolerance(new Translation3d(0.5, 1.2, 0.3))
                // Set a callback to run when the fuel hits the target
                .withHitTargetCallBack(() -> {
                    System.out.println("Hit hub, +1 point!");
                    simulatedScore += 1;
                    simulatedScoreEntry.setInteger(simulatedScore);
                    // This is roughly in the neutral zone.
                    // We add an arbitrary random number to it, because game pieces
                    // that spawn on top of each other in MapleSim are a little weird.
                    SimulatedArena.getInstance().addGamePiece(
                            new RebuiltFuelOnField(new Translation2d(random.nextDouble(10.8, 10.9),
                                    random.nextDouble(3, 4.5))));
                });

        fuelOnFly
                // Configure the fuel projectile to be "on the field" upon touching the
                // ground

                .enableBecomesGamePieceOnFieldAfterTouchGround();

        // Add the projectile to the simulated arena
        SimulatedArena.getInstance().addGamePieceProjectile(fuelOnFly);
    }

    /*
     * The timestamp, in milliseconds, of when we shot the last piece of fuel.
     * This allows us to throttle how many can be shot over time.
     */
    private long lastSentFuel = -1;

    public void simulationInit() {
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public void simulationPeriodic() {
        var shooterOpt = shooterSystem.get();
        if (shooterOpt.isEmpty()) {
            return;
        }

        var intakeOpt = intakeSystem.get();
        if (intakeOpt.isEmpty()) {
            return;
        }

        var beltOpt = beltSystem.get();
        if (beltOpt.isEmpty()) {
            return;
        }

        var indexerOpt = indexerSystem.get();
        if (indexerOpt.isEmpty()) {
            return;
        }

        var shooter = shooterOpt.get();
        var intake = intakeOpt.get();
        var shooterRPM = shooter.getGoalRPM();
        long currentTime = System.currentTimeMillis();

        if (shooterRPM > 0
                && (beltOpt.get().getSpeed() > 0)
                && (indexerOpt.get().getSpeed() > 0)
                && (currentTime - lastSentFuel >= 200)) {
            var intakeSimulation = intake.getIntakeSimulation();
            driveSystem.ifEnabled(drive -> {
                // TODO: Peter: We should get this to precisely match our CAD
                if (intakeSimulation.obtainGamePieceFromIntake()) {
                    addFuelProjectile(drive, shooterRPM, -0.1);
                }
                if (intakeSimulation.obtainGamePieceFromIntake()) {
                    addFuelProjectile(drive, shooterRPM, 0.1);
                }
                lastSentFuel = currentTime;
            });

        }

        Pose3d[] fuelPoses = SimulatedArena.getInstance()
                .getGamePiecesArrayByType("Fuel");
        fuelPosePublisher.accept(fuelPoses);
        SimulatedArena.getInstance().simulationPeriodic();
    }
}

package frc.robot.subsystems.drive;

import java.io.File;
import java.util.Arrays;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.services.vision.VisionService;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

public class SimulatedSwerve extends Swerve {
    /*
     * Field containing the simulated pose based on YagSL.
     * This is for use in AdvantageScope, and doesn't work when the robot
     * isn't in simulation mode. Use the "Estimated Pose" (based on vision)
     * for real matches.
     */
    private final Field2d simField = new Field2d();

    private final SwerveDriveSimulation mapleSimDrive;

    public SwerveDriveSimulation getMapleSimDrive() {
        return mapleSimDrive;
    }

    public SimulatedSwerve(File directory, VisionService vision) {
        super(directory, vision);
        Shuffleboard.getTab("Console")
                .add("Simulated Pose", simField)
                .withSize(4, 2)
                .withWidget(BuiltInWidgets.kField);
        mapleSimDrive = new SwerveDriveSimulation(
                DriveTrainSimulationConfig.Default()
                        .withRobotMass(Kilograms.of(54))
                        .withBumperSize(Meters.of(0.9), Meters.of(0.9))
                        .withCustomModuleTranslations(
                                Arrays.stream(swerveDrive.getModules())
                                        .map(m -> m.configuration.moduleLocation)
                                        .toArray(Translation2d[]::new)),
                swerveDrive.getPose());

        SimulatedArena.getInstance().addDriveTrainSimulation(mapleSimDrive);
    }

    @Override
    public void simulationPeriodic() {
        swerveDrive.getSimulationDriveTrainPose()
                .ifPresent(pose -> {
                    simField.setRobotPose(pose);
                    // Keep the org.ironmaple drivetrain in sync with YAGSL's sim
                    mapleSimDrive.setSimulationWorldPose(pose);
                });
    }
}

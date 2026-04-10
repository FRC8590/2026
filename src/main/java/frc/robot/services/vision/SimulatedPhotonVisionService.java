package frc.robot.services.vision;

import java.util.Arrays;

import org.photonvision.simulation.VisionSystemSim;

import lib.woodsonrobotics.vision.photon.PhotonVisionCamera;
import lib.woodsonrobotics.vision.photon.SimulatedPhotonVisionCamera;
import swervelib.SwerveDrive;

/**
 * The service for vision simulations. Unlike VisionService,
 * which uses the generic Camera API, this only works with PhotonVision!
 */
public class SimulatedPhotonVisionService extends VisionService {
    public final VisionSystemSim photonVisionSim;

    public SimulatedPhotonVisionService(PhotonVisionCamera[] camerasArray) {
        this(camerasArray, new VisionSystemSim("main"));
    }

    private SimulatedPhotonVisionService(PhotonVisionCamera[] camerasArray, VisionSystemSim sim) {
        super(Arrays.stream(camerasArray)
                .map(camera -> new SimulatedPhotonVisionCamera(camera, sim))
                .toArray(SimulatedPhotonVisionCamera[]::new));

        photonVisionSim = sim;
    }

    public void updateSwerveEstimation(SwerveDrive swerveDrive) {
        photonVisionSim.update(swerveDrive.getSimulationDriveTrainPose().get());
        super.updateSwerveEstimation(swerveDrive);
    }
}
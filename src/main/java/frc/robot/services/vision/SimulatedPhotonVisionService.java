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
    public static final VisionSystemSim photonVisionSim = new VisionSystemSim("main");

    public SimulatedPhotonVisionService(PhotonVisionCamera[] camerasArray) {
        super(Arrays.stream(camerasArray)
                .map(camera -> new SimulatedPhotonVisionCamera(camera, photonVisionSim))
                .toArray(SimulatedPhotonVisionCamera[]::new));
    }

    public void updateSwerveEstimation(SwerveDrive swerveDrive) {
        photonVisionSim.update(swerveDrive.getSimulationDriveTrainPose().get());
        super.updateSwerveEstimation(swerveDrive);
    }
}

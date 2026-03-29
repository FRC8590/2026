package lib.woodsonrobotics.vision.photon;

import java.util.Optional;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Robot;
import lib.woodsonrobotics.vision.EstimatedPose;

public class SimulatedPhotonVisionCamera extends PhotonVisionCamera {
    public static final VisionSystemSim visionSim = new VisionSystemSim("main");
    private final PhotonCameraSim cameraSim;

    public SimulatedPhotonVisionCamera(String name, AprilTagFieldLayout fieldLayout, Transform3d robotToCamTransform,
            Resolution cameraResolution) {
        super(name, fieldLayout, robotToCamTransform, cameraResolution);
        visionSim.addAprilTags(fieldLayout);

        SimCameraProperties cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(cameraResolution.width(), cameraResolution.height(), Rotation2d.fromDegrees(100));
        cameraProp.setCalibError(0.25, 0.08);
        cameraProp.setFPS(20);
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);
        cameraSim = new PhotonCameraSim(camera, cameraProp);
        cameraSim.enableDrawWireframe(true);
        visionSim.addCamera(cameraSim, robotToCamTransform);
    }

    public SimulatedPhotonVisionCamera(PhotonVisionCamera camera) {
        this(camera.getName(), camera.getFieldLayout(), camera.getRobotToCamera(), camera.getResolution());
    }

    @Override
    public void refresh() {
        resultsLock.lock();
        try {
            resultsList = cameraSim.getCamera().getAllUnreadResults();
            resultsList.sort((PhotonPipelineResult a, PhotonPipelineResult b) -> {
                return a.getTimestampSeconds() >= b.getTimestampSeconds() ? 1 : -1;
            });
        } finally {
            resultsLock.unlock();
        }
    }

    @Override
    public Optional<EstimatedPose> getEstimatedGlobalPose() {
        var result = super.getEstimatedGlobalPose();
        if (result != null && Robot.isSimulation()) {
            Field2d debugField = visionSim.getDebugField();
            result.ifPresentOrElse(
                    est -> debugField
                            .getObject("VisionEstimation")
                            .setPose(result.get().toPose2d()),
                    () -> {
                        debugField.getObject("VisionEstimation").setPoses();
                    });
        }
        return result;
    }
}

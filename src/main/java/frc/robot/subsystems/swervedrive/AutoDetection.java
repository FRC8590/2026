package frc.robot.subsystems.swervedrive;

import java.awt.image.BufferedImage;
import java.io.IOException;
import java.io.InputStream;
import java.net.URL;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import javax.imageio.ImageIO;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Seconds;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class AutoDetection {



  public AutoDetection(){
      
  }


  public enum Cameras
  {

    /**
     * Right Camera
     */
    RIGHT_CAM("right",
              new Rotation3d(0, 0, Units.degreesToRadians(0)),
              new Translation3d(Units.inchesToMeters(1),
                                Units.inchesToMeters(13.5),
                                Units.inchesToMeters(8.25)),
              VecBuilder.fill(4, 4, 8), VecBuilder.fill(0.5, 0.5, 1));

    /**
     * Latency alert to use when high latency is detected.
     */
    public final  Alert                        latencyAlert;
    /**
     * Camera instance for comms.
     */
    public  PhotonCamera                 camera;
    /**
     * Transform of the camera rotation and translation relative to the center of the robot
     */
    private final Transform3d                  robotToCamTransform;
    /**
     * Current standard deviations used.
     */
    public        Matrix<N3, N1>               curStdDevs;
    /**
     * Simulated camera instance which only exists during simulations.
     */
    public        PhotonCameraSim              cameraSim;
    /**
     * Results list to be updated periodically and cached to avoid unnecessary queries.
     */
    public        List<PhotonPipelineResult>   resultsList       = new ArrayList<>();
    /**
     * Last read from the camera timestamp to prevent lag due to slow data fetches.
     */
    private       double                       lastReadTimestamp = Microseconds.of(NetworkTablesJNI.now()).in(Seconds);

    private boolean hasSetOffset = false;

    private double timerOffset = 0;
    /**
     * Construct a Photon Camera class with help. Standard deviations are fake values, experiment and determine
     * estimation noise on an actual robot.
     *
     * @param name                  Name of the PhotonVision camera found in the PV UI.
     * @param robotToCamRotation    {@link Rotation3d} of the camera.
     * @param robotToCamTranslation {@link Translation3d} relative to the center of the robot.
     * @param singleTagStdDevs      Single AprilTag standard deviations of estimated poses from the camera.
     * @param multiTagStdDevsMatrix Multi AprilTag standard deviations of estimated poses from the camera.
     */
    Cameras(String name, Rotation3d robotToCamRotation, Translation3d robotToCamTranslation,
            Matrix<N3, N1> singleTagStdDevs, Matrix<N3, N1> multiTagStdDevsMatrix)
    {
      latencyAlert = new Alert("'" + name + "' Camera is experiencing high latency.", AlertType.kWarning);

      camera = null;
      try{
        camera = new PhotonCamera(name);
      }catch(Exception e){
        System.out.println(e);
      }

      // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
      robotToCamTransform = new Transform3d(robotToCamTranslation, robotToCamRotation);


    }

    /**
     * Get the latest result from the current cache.
     *
     * @return Empty optional if nothing is found. Latest result if something is there.
     */
    public Optional<PhotonPipelineResult> getLatestResult()
    {
      return resultsList.isEmpty() ? Optional.empty() : Optional.of(resultsList.get(0));
    }





  }


    
  /***
   * @return Returns a boolean array of (0, top left; 1, bottom left; 2, top right; 3, bottom right) if the camera input sees the side
   * @throws IOException 
   */
  public static void coralScored() throws IOException{

    URL url = new URL("http://photonvision.local:1181");
    InputStream stream = url.openStream();
    BufferedImage frame = ImageIO.read(stream); // Read a single frame

  }


    
}

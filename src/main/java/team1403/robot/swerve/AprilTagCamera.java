
package team1403.robot.swerve;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import dev.doglog.DogLog;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.robot.Constants;
import team1403.robot.Robot;

public class AprilTagCamera extends SubsystemBase {
  private final PhotonCamera m_camera;
  private PhotonCameraSim m_cameraSim;
  private PhotonPoseEstimator m_poseEstimator;
  private Supplier<Transform3d> m_cameraTransform;
  private EstimatedRobotPose m_estPos;
  private Supplier<Pose3d> m_referencePose;
  private static final Matrix<N4, N1> kDefaultStdv = VecBuilder.fill(2, 2, 4, 999999);
  private static final boolean kExtraVisionDebugInfo = true;

  public AprilTagCamera(String cameraName, Supplier<Transform3d> cameraTransform, Supplier<Pose3d> referenceSupplier) {
    // Photonvision
    // PortForwarder.add(5800, 
    // "photonvision.local", 5800);
    m_camera = new PhotonCamera(cameraName);

    if (Robot.isSimulation()) {
      SimCameraProperties cameraProp = new SimCameraProperties();

      // A 640 x 480 camera with a 57 degree diagonal FOV.
      cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(57));
      // Approximate detection noise with average and standard deviation error in pixels.
      cameraProp.setCalibError(0.25, 0.08);
      // Set the camera image capture framerate (Note: this is limited by robot loop rate).
      cameraProp.setFPS(30);
      // The average and standard deviation in milliseconds of image data latency.
      cameraProp.setAvgLatencyMs(35);
      cameraProp.setLatencyStdDevMs(5);

      m_cameraSim = new PhotonCameraSim(m_camera, cameraProp);

      VisionSimUtil.addCamera(m_cameraSim, cameraTransform.get());
    }

    // 0: April Tags


    // 1: Reflective Tape
    m_camera.setPipelineIndex(0);

    m_poseEstimator = new PhotonPoseEstimator(Constants.Vision.kFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraTransform.get());
    m_poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);

    m_estPos = null;
    m_referencePose = referenceSupplier;
    m_cameraTransform = cameraTransform;
    
    //keep this enabled even without debug mode for logging purposes
    Constants.kDebugTab.addBoolean(m_camera.getName() + " connected", () -> m_camera.isConnected());
  }

  @Override
  public String getName() {
    return m_camera.getName();
  }

  public boolean hasPose() {
    return m_estPos != null;
  }

  public Pose3d getPose() {
    if(hasPose())
    {
      return m_estPos.estimatedPose;
    }
    return null;
  }

  //gets the timestamp of the latest pose
  public double getTimestamp() {
    if(hasPose())
    {
      return m_estPos.timestampSeconds;
    }
    return -1;
  }

  public List<PhotonTrackedTarget> getTargets() {
    if(hasPose())
    {
      return m_estPos.targetsUsed;
    }
    return new ArrayList<>();
  }

  private double getTagAreas() {
    double ret = 0;
    if(!hasPose()) return 0;
    for(PhotonTrackedTarget t : getTargets()) {
      ret += t.getArea();
    }
    return ret;
  }

  public Matrix<N4, N1> getEstStdv() {
    return kDefaultStdv.div(getTagAreas());
  }

  //TODO: return false for bad estimates
  public boolean checkVisionResult() {

    if(getTagAreas() < 0.3) return false;

    if(getPose().getZ() > 1){
      return false;
    }

    if(getTargets().size() == 1) {
      if(getTargets().get(0).getPoseAmbiguity() > 0.6)
        return false;
    }

    return true;
  }

  private final ArrayList<Pose3d> m_visionTargets = new ArrayList<>();
  private final ArrayList<Translation2d> m_corners = new ArrayList<>();
  private static final Transform3d kZeroTransform = new Transform3d();

  @Override
  public void periodic() {

    m_poseEstimator.setReferencePose(m_referencePose.get());
    m_poseEstimator.setRobotToCameraTransform(m_cameraTransform.get());

    if(m_cameraSim != null) {
      VisionSimUtil.adjustCamera(m_cameraSim, m_cameraTransform.get());
    }


    for(PhotonPipelineResult result : m_camera.getAllUnreadResults())
    {
      //fixme: indentation
        m_estPos = m_poseEstimator.update(result).orElse(null);

        DogLog.log(m_camera.getName() + "/Target Visible", result.hasTargets());

        if(kExtraVisionDebugInfo)
        {
          Pose3d robot_pose3d = m_referencePose.get();
          Pose3d robot_pose_transformed = robot_pose3d.transformBy(m_cameraTransform.get());
          double[] ambiguities = new double[getTargets().size()];

          DogLog.log(m_camera.getName() + "/Camera Transform", robot_pose_transformed);

          m_visionTargets.clear();
          m_corners.clear();

          for(int i = 0; i < getTargets().size(); i++) {
            PhotonTrackedTarget t = getTargets().get(i);
            Transform3d trf = t.getBestCameraToTarget();
            if(trf.equals(kZeroTransform)) continue;

            m_visionTargets.add(robot_pose_transformed.transformBy(trf));
            ambiguities[i] = t.getPoseAmbiguity();
            for(TargetCorner c : t.getDetectedCorners())
              m_corners.add(new Translation2d(c.x, c.y));
          }

          DogLog.log(m_camera.getName() + "/Vision Targets", m_visionTargets.toArray(new Pose3d[m_visionTargets.size()]));
          DogLog.log(m_camera.getName() + "/Corners", m_corners.toArray(new Translation2d[m_corners.size()]));
          DogLog.log(m_camera.getName() + "/PoseAmbiguity", ambiguities.clone());
        }

        DogLog.log(m_camera.getName() + "/hasPose", hasPose());
        
        if(hasPose())
        {
          DogLog.log(m_camera.getName() + "/Combined Area", getTagAreas());
          DogLog.log(m_camera.getName() + "/Pose3d", getPose());
        }
      }
    }
  }


package team1403.robot.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.robot.Constants;
import team1403.robot.Robot;

public class AprilTagCamera extends SubsystemBase implements ITagCamera {
  private final PhotonCamera m_camera;
  private final PhotonCameraSim m_cameraSim;
  private final PhotonPoseEstimator m_poseEstimator;
  private final Supplier<Transform3d> m_cameraTransform;
  private EstimatedRobotPose m_estPos;
  private final Supplier<Pose2d> m_referencePose;
  private final DoubleSupplier m_poseTimestamp;
  private final Alert m_cameraAlert;
  private static final Matrix<N3, N1> kDefaultStdv = VecBuilder.fill(2, 2, 3);
  
  private final VisionData m_returnedData = new VisionData();

  public AprilTagCamera(String cameraName, Supplier<Transform3d> cameraTransform, DoubleSupplier poseTimestamp, Supplier<Pose2d> referenceSupplier) {
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
    } else {
      m_cameraSim = null;
    }
    m_camera.setPipelineIndex(0);

    m_poseEstimator = new PhotonPoseEstimator(Constants.Vision.kFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraTransform.get());
    m_poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);

    m_estPos = null;
    m_referencePose = referenceSupplier;
    m_cameraTransform = cameraTransform;
    m_poseTimestamp = poseTimestamp;

    m_cameraAlert = new Alert("Photon Camera " + cameraName + " Disconnected!", AlertType.kError);
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

  private List<PhotonTrackedTarget> getTargets() {
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

  public Matrix<N3, N1> getEstStdv() {
    return kDefaultStdv.div(getTagAreas());
  }

  public boolean checkVisionResult() {
    if(!hasPose()) return false;

    if(getTagAreas() < 0.3) return false;

    if(getPose().getZ() > 1) return false;

    if(getTargets().size() == 1) {
      if(getTargets().get(0).getPoseAmbiguity() > 0.6)
        return false;
    }

    return true;
  }

  private void copyVisionData() {
    m_returnedData.pose = getPose();
    m_returnedData.stdv = getEstStdv();
    m_returnedData.timestamp = getTimestamp();
  }

  public void refreshEstimate(Consumer<VisionData> data) {
    if(checkVisionResult()) {
      copyVisionData();
      data.accept(m_returnedData);
    }
  }

  private final ArrayList<Pose3d> m_visionTargets = new ArrayList<>();
  private final ArrayList<Translation2d> m_corners = new ArrayList<>();
  private static final Transform3d kZeroTransform = new Transform3d();

  @Override
  public void periodic() {

    m_poseEstimator.setReferencePose(m_referencePose.get());
    //think about if we want to use the trig solver or constrained solve pnp
    m_poseEstimator.addHeadingData(m_poseTimestamp.getAsDouble(), m_referencePose.get().getRotation());
    m_poseEstimator.setRobotToCameraTransform(m_cameraTransform.get());

    if(m_cameraSim != null) {
      VisionSimUtil.adjustCamera(m_cameraSim, m_cameraTransform.get());
    }

    //replaced this with an alert
    // SmartDashboard.putBoolean(m_camera.getName() + " connected", m_camera.isConnected());

    m_cameraAlert.set(!m_camera.isConnected());

    for(PhotonPipelineResult result : m_camera.getAllUnreadResults())
    {
      //fixme: indentation
        m_estPos = m_poseEstimator.update(result).orElse(null);

        Logger.recordOutput(m_camera.getName() + "/Target Visible", result.hasTargets());

        if(Constants.Vision.kExtraVisionDebugInfo)
        {
          Pose3d robot_pose3d = new Pose3d(m_referencePose.get());
          Pose3d robot_pose_transformed = robot_pose3d.transformBy(m_cameraTransform.get());
          double ambiguity = 0;

          Logger.recordOutput(m_camera.getName() + "/Camera Transform", robot_pose_transformed);

          m_visionTargets.clear();
          m_corners.clear();

          for(int i = 0; i < getTargets().size(); i++) {
            PhotonTrackedTarget t = getTargets().get(i);
            Transform3d trf = t.getBestCameraToTarget();
            if(trf.equals(kZeroTransform)) continue;

            m_visionTargets.add(robot_pose_transformed.transformBy(trf));
            for(TargetCorner c : t.getDetectedCorners())
              m_corners.add(new Translation2d(c.x, c.y));
          }

          if(getTargets().size() == 1) ambiguity = getTargets().get(0).poseAmbiguity;

          Logger.recordOutput(m_camera.getName() + "/Vision Targets", m_visionTargets.toArray(new Pose3d[m_visionTargets.size()]));
          Logger.recordOutput(m_camera.getName() + "/Corners", m_corners.toArray(new Translation2d[m_corners.size()]));
          Logger.recordOutput(m_camera.getName() + "/PoseAmbiguity", ambiguity);
        }

        Logger.recordOutput(m_camera.getName() + "/hasPose", hasPose());
        
        if(hasPose())
        {
          Logger.recordOutput(m_camera.getName() + "/Combined Area", getTagAreas());
          Logger.recordOutput(m_camera.getName() + "/Pose3d", getPose());
        }
      }
    }
  }


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
  private final PhotonPoseEstimator m_trigSolveEstimator;
  private final Supplier<Transform3d> m_cameraTransform;
  private EstimatedRobotPose m_estPos;
  private EstimatedRobotPose m_estTrigPos;
  private final Supplier<Pose2d> m_referencePose;
  private final DoubleSupplier m_poseTimestamp;
  private final DoubleSupplier m_angularVelocity;
  private final Alert m_cameraAlert;
  private final Matrix<N3, N1> kDefaultStdv;
  private final Matrix<N3, N1> kDefaultStdvTrig;
  private final boolean kTrigSolveEnabled;
  
  private final VisionData m_returnedData = new VisionData();

  public AprilTagCamera(VisionConfigurator config) {
    // Photonvision
    // PortForwarder.add(5800, 
    // "photonvision.local", 5800);
    m_camera = new PhotonCamera(config.getName());

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

      VisionSimUtil.addCamera(m_cameraSim, config.getTransform3d().get());
    } else {
      m_cameraSim = null;
    }
    m_camera.setPipelineIndex(0);

    m_poseEstimator = new PhotonPoseEstimator(Constants.Vision.kFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, config.getTransform3d().get());
    m_poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);

    m_trigSolveEstimator = new PhotonPoseEstimator(Constants.Vision.kFieldLayout, PoseStrategy.PNP_DISTANCE_TRIG_SOLVE, config.getTransform3d().get());

    m_estPos = null;
    m_referencePose = config.getRobotPose();
    m_cameraTransform = config.getTransform3d();
    m_poseTimestamp = config.getPoseTimestamp();
    kDefaultStdv = config.getDeviations();
    kDefaultStdvTrig = config.getDeviationsTrig();
    kTrigSolveEnabled = config.getTrigSolveEnabled();
    m_angularVelocity = config.getYawRate();

    m_cameraAlert = new Alert("Photon Camera " + m_camera.getName() + " Disconnected!", AlertType.kError);
  }

  @Override
  public String getName() {
    return m_camera.getName();
  }

  public boolean hasPose(EstimatedRobotPose pos) {
    return pos != null;
  }

  public Pose3d getPose(EstimatedRobotPose pos) {
    if(hasPose(pos))
    {
      return pos.estimatedPose;
    }
    return null;
  }

  //gets the timestamp of the latest pose
  public double getTimestamp(EstimatedRobotPose pos) {
    if(hasPose(pos))
    {
      return pos.timestampSeconds;
    }
    return -1;
  }

  private List<PhotonTrackedTarget> getTargets(EstimatedRobotPose pos) {
    if(hasPose(pos))
    {
      return pos.targetsUsed;
    }
    return new ArrayList<>();
  }

  private double getTagAreas(EstimatedRobotPose pos) {
    double ret = 0;
    if(!hasPose(pos)) return 0;
    for(PhotonTrackedTarget t : getTargets(pos)) {
      ret += t.getArea();
    }
    return ret;
  }

  public Matrix<N3, N1> getEstStdv(EstimatedRobotPose pos) {
    if(pos.strategy == PoseStrategy.PNP_DISTANCE_TRIG_SOLVE)
    {
      return kDefaultStdvTrig.div(getTagAreas(pos));
    }

    return kDefaultStdv.div(getTagAreas(pos));
  }

  public boolean checkVisionResult(EstimatedRobotPose pos) {
    if(!hasPose(pos)) return false;

    if(getTagAreas(pos) < 0.3) return false;

    if(getPose(pos).getZ() > 1) return false;

    if(getTargets(pos).size() == 1) {
      if(getTargets(pos).get(0).getPoseAmbiguity() > 0.6)
        return false;
    }

    return true;
  }

  private void copyVisionData(EstimatedRobotPose pos) {
    m_returnedData.pose = getPose(pos);
    m_returnedData.stdv = getEstStdv(pos);
    m_returnedData.timestamp = getTimestamp(pos);
  }

  public void refreshEstimate(Consumer<VisionData> data) {
    if (checkVisionResult(m_estPos)) {
      copyVisionData(m_estPos);
      data.accept(m_returnedData);
    }
    if (checkVisionResult(m_estTrigPos)) {
      copyVisionData(m_estTrigPos);
      data.accept(m_returnedData);
    }
  }

  private final ArrayList<Pose3d> m_visionTargets = new ArrayList<>();
  private static final Transform3d kZeroTransform = new Transform3d();

  @Override
  public void periodic() {

    if (m_referencePose != null) {
      m_poseEstimator.setReferencePose(m_referencePose.get());
      m_trigSolveEstimator.setReferencePose(m_referencePose.get());
    }
    //think about if we want to use the trig solver or constrained solve pnp
    if (kTrigSolveEnabled) {
      m_trigSolveEstimator.addHeadingData(m_poseTimestamp.getAsDouble(), m_referencePose.get().getRotation());
      m_poseEstimator.addHeadingData(m_poseTimestamp.getAsDouble(), m_referencePose.get().getRotation());
    }

    m_poseEstimator.setRobotToCameraTransform(m_cameraTransform.get());
    m_trigSolveEstimator.setRobotToCameraTransform(m_cameraTransform.get());

    if(m_cameraSim != null) {
      VisionSimUtil.adjustCamera(m_cameraSim, m_cameraTransform.get());
    }

    //replaced this with an alert
    // SmartDashboard.putBoolean(m_camera.getName() + " connected", m_camera.isConnected());

    m_cameraAlert.set(!m_camera.isConnected());

    Pose3d robot_pose3d = new Pose3d(m_referencePose.get());
    Pose3d robot_pose_transformed = robot_pose3d.transformBy(m_cameraTransform.get());

    Logger.recordOutput(m_camera.getName() + "/Camera Transform", robot_pose_transformed);

    for(PhotonPipelineResult result : m_camera.getAllUnreadResults())
    {
      //fixme: indentation
        m_estPos = m_poseEstimator.update(result).orElse(null);
        if (kTrigSolveEnabled && Math.abs(m_angularVelocity.getAsDouble()) < 0.5)
          m_estTrigPos = m_trigSolveEstimator.update(result).orElse(null);
        else 
          m_estTrigPos = null;

        Logger.recordOutput(m_camera.getName() + "/Target Visible", result.hasTargets());

        if(Constants.Vision.kExtraVisionDebugInfo)
        {
          List<PhotonTrackedTarget> targets = getTargets(m_estPos);
          double ambiguity = 0;

          m_visionTargets.clear();

          for(int i = 0; i < targets.size(); i++) {
            PhotonTrackedTarget t = getTargets(m_estPos).get(i);
            Transform3d trf = t.getBestCameraToTarget();
            if(trf.equals(kZeroTransform)) continue;

            m_visionTargets.add(robot_pose_transformed.transformBy(trf));
          }

          if(targets.size() == 1) ambiguity = targets.get(0).poseAmbiguity;

          Logger.recordOutput(m_camera.getName() + "/Vision Targets", m_visionTargets.toArray(new Pose3d[m_visionTargets.size()]));
          Logger.recordOutput(m_camera.getName() + "/PoseAmbiguity", ambiguity);
        }

        Logger.recordOutput(m_camera.getName() + "/hasPose", hasPose(m_estPos) || hasPose(m_estTrigPos));
        
        if (hasPose(m_estPos))
        {
          Logger.recordOutput(m_camera.getName() + "/Combined Area", getTagAreas(m_estPos));
          Logger.recordOutput(m_camera.getName() + "/Pose3d", getPose(m_estPos));
        }
        if (hasPose(m_estTrigPos))
        {
          Logger.recordOutput(m_camera.getName() + "/Pose3dTrig", getPose(m_estTrigPos));
        }
      }
    }
  }

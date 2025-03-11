package team1403.robot.vision;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;
import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.robot.Constants;

//import frc.robot.LimelightHelpers; 

public class LimelightWrapper extends SubsystemBase implements ITagCamera {
    private final String m_name;
    private final Supplier<Rotation3d> m_imuRotation;
    private final Supplier<Transform3d> m_camTransform;
    private LimelightHelpers.PoseEstimate m_poseEstimate;
    private final static Matrix<N3, N1> kDefaultStdv = VecBuilder.fill(2, 2, 3);
    private final Alert m_camDisconnected;
    private final DoubleSubscriber m_latencySubscriber;
    

    public LimelightWrapper(String name, Supplier<Transform3d> cameraTransform, Supplier<Rotation3d> imuRotation) {
        m_name = name.toLowerCase(); //hostname must be lowercase
        m_imuRotation = imuRotation;
        m_camTransform = cameraTransform;
        m_poseEstimate = null;

        m_latencySubscriber = LimelightHelpers.getLimelightNTTable(m_name)
            .getDoubleTopic("tl").subscribe(0.0);
        m_camDisconnected = new Alert("Limelight " + m_name + " Disconnected!", AlertType.kError);
    }
    
    @Override
    public boolean hasPose() {
        return LimelightHelpers.validPoseEstimate(m_poseEstimate);
    }

    @Override
    public Pose3d getPose() {
        if (hasPose()){
            return m_poseEstimate.pose;
        } else {
            return null;
        }
    }

    @Override
    public double getTimestamp() {
        if (!hasPose()) return -1;
        return m_poseEstimate.timestampSeconds;
    }

    @Override
    public Matrix<N3, N1> getEstStdv() {
        return kDefaultStdv;
    }

    
    private double getTagAreas() {
        if(!hasPose()) return 0;
        return m_poseEstimate.avgTagArea * m_poseEstimate.tagCount;
    }

    private LimelightHelpers.RawFiducial[] getTargets() {
        if(hasPose())
        {
            return m_poseEstimate.rawFiducials;
        }
        return null;
    }

    @Override
    public boolean checkVisionResult() {
        if(!hasPose()) return false;

        // area units are a bit different, so disable this check
        // if(getTagAreas() < 0.3) return false;

        if(getPose().getZ() > 1) return false;
        if(getPose().getZ() < -0.5) return false;

        if(getTargets().length == 1) {
            if(getTargets()[0].ambiguity > 0.6) {
                return false;
            }
        }
        
        return true;
    }

    private boolean isConnected() {
        long dt_us = RobotController.getFPGATime() - m_latencySubscriber.getLastChange();
        double dt = Seconds.convertFrom(dt_us, Microseconds);
        return dt < 0.25;
    }

    private final ArrayList<Pose3d> targets = new ArrayList<>();
    
    @Override
    public void periodic() {    
        LimelightHelpers.SetRobotOrientation(m_name, m_imuRotation.get());
        LimelightHelpers.setCameraPose_RobotSpace(m_name, m_camTransform.get());
        m_poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(m_name);

        m_camDisconnected.set(!isConnected());
        
        Logger.recordOutput(m_name + "/hasPose", hasPose());
        targets.clear();

        if(hasPose()) {
            Logger.recordOutput(m_name + "/pose3d", m_poseEstimate.pose);
            Logger.recordOutput(m_name + "/tagArea", getTagAreas());
            Logger.recordOutput(m_name + "/cameraTransform", m_camTransform.get());

            /* if the pose estimate is valid then getTargets() != null */
            LimelightHelpers.RawFiducial[] fiducials = getTargets();
            Logger.recordOutput(m_name + "/ambiguity", fiducials.length == 1 ? fiducials[0].ambiguity : 0);
          
            if(Constants.Vision.kExtraVisionDebugInfo)
            {
                for(int i = 0; i < fiducials.length; i++)
                {
                    LimelightHelpers.RawFiducial f = fiducials[i];
                    Optional<Pose3d> pose = Constants.Vision.kFieldLayout.getTagPose(f.id);
                    if(pose.isPresent()) targets.add(pose.get());
                }
                Logger.recordOutput(m_name + "/visionTargets", targets.toArray(new Pose3d[targets.size()]));
            }
        }
        
        
    }
}

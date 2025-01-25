package team1403.robot.vision;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightWrapper extends SubsystemBase implements ITagCamera {
    private String m_name;
    private Supplier<Rotation3d> m_imuRotation;
    private Supplier<Transform3d> m_camTransform;
    private LimelightHelpers.PoseEstimate m_poseEstimate;
    private final static Matrix<N3, N1> kDefaultStdv = VecBuilder.fill(2, 2, 999999);
    

    public LimelightWrapper(String name, Supplier<Transform3d> cameraTransform, Supplier<Rotation3d> imuRotation) {
        m_name = name.toLowerCase(); //hostname must be lowercase
        m_imuRotation = imuRotation;
        m_camTransform = cameraTransform;
        m_poseEstimate = null;
    }
    
    @Override
    public boolean hasPose() {
        return m_poseEstimate != null;
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
        //TODO: change it based on the confidence
        return kDefaultStdv.div(1);
    }

    
    private double getTagAreas() {
        if (!hasPose()) return 0;
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

        if(getTagAreas() < 0.3) return false;

        if(getPose().getZ() > 1) return false;

        LimelightHelpers.RawFiducial[] fiducials = getTargets();

        if(fiducials != null && fiducials.length == 1) {
            if(fiducials[0].ambiguity > 0.6) {
                return false;
            }
        }

        return true;
    }
    
    @Override
    public void periodic() {
        LimelightHelpers.setCameraPose_RobotSpace(m_name, m_camTransform.get());
        LimelightHelpers.SetRobotOrientation(m_name, m_imuRotation.get());
        m_poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(m_name);
        

        if(hasPose()) {
            Logger.recordOutput("limelights/" + m_name + "/pose3d", getPose());
            Logger.recordOutput("limelights/" + m_name + "/area", getTagAreas());
            Logger.recordOutput("limelights/" + m_name + "/valid", checkVisionResult());
        }
        
        
    }
}
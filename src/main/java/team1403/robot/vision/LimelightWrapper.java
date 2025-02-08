package team1403.robot.vision;

import static edu.wpi.first.units.Units.Rotation;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.robot.Constants;

//import frc.robot.LimelightHelpers; 

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
    //TODO: implement these functions
    

    @Override
    public boolean hasPose() {
        return m_poseEstimate != null && LimelightHelpers.validPoseEstimate(m_poseEstimate);
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
        return kDefaultStdv.div(1.0);
    }

    
    private double getTagAreas() {
        double ret = 0;
        if(!hasPose()) return 0;
        for(LimelightHelpers.RawFiducial t : getTargets()) {
          ret += t.ta;
        }
        return ret;
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

        if(getTargets().length == 1) {
            if(getTargets()[0].ambiguity > 0.6) {
                return false;
            }
            return false;
        }
        else {
            return true;
        }
    }
    
    @Override
    public void periodic() {    
        LimelightHelpers.SetRobotOrientation(m_name, m_imuRotation.get());
        LimelightHelpers.setCameraPose_RobotSpace(m_name, m_camTransform.get());
        m_poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(m_name);
        
        if(hasPose()) {
            Logger.recordOutput(m_name + "/pose3d", m_poseEstimate.pose);
            Logger.recordOutput(m_name + "/tagArea", getTagAreas());
        }
        
        
    }
}

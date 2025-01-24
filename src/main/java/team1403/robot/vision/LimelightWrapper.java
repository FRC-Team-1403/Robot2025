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
    private EstimatedRobotPose m_estPos;
    private LimelightHelpers.PoseEstimate m_poseEstimate;
    private final static Matrix<N3, N1> kDefaultStdv = VecBuilder.fill(2, 2, 999999);
    

    public LimelightWrapper(String name, Supplier<Transform3d> cameraTransform, Supplier<Rotation3d> imuRotation) {
        m_name = name.toLowerCase(); //hostname must be lowercase
        m_imuRotation = imuRotation;
        m_camTransform = cameraTransform;
        m_poseEstimate = null;
        m_estPos = null;

    }
    //TODO: implement these functions
    

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
        return kDefaultStdv.div(0);
    }

    
    private double getTagAreas() {
        double ret = 0;
        if(!hasPose()) return 0;
        for(PhotonTrackedTarget t : getTargets()) {
          ret += t.getArea();
        }
        return ret;
    }

    private List<PhotonTrackedTarget> getTargets() {
        if(hasPose())
        {
            return m_estPos.targetsUsed;
        }
        return new ArrayList<>();
    }

    @Override
    public boolean checkVisionResult() {
        if(!hasPose()) return false;

        if(getTagAreas() < 0.3) return false;

        if(getPose().getZ() > 1) return false;

        if(getTargets().size() == 1) {
            if(getTargets().get(0).getPoseAmbiguity() > 0.6) {
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
        m_poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(m_name);
        

        if(hasPose()) {
            Logger.recordOutput(m_name + "/pose3d", m_poseEstimate.pose);
        }
        
        
    }
}

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

    private final String m_name;
    private final Supplier<Rotation3d> m_imuRotation;
    private final Supplier<Transform3d> m_camTransform;
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
        return false;
    }

    @Override
    public Pose3d getPose() {
        if (!hasPose()) return null;
        return null;
    }

    @Override
    public double getTimestamp() {
        if (!hasPose()) return -1;
        return -1;
    }

    @Override
    public Matrix<N3, N1> getEstStdv() {
        //TODO: change it based on the confidence
        return kDefaultStdv;
    }

    @Override
    public boolean checkVisionResult() {
        if(!hasPose()) return false;

        //TODO: implement more accuracy things


        return true;
    }
    
    @Override
    public void periodic() {
 
    }
}

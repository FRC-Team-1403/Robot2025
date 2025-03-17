package team1403.robot.vision;

import java.util.function.Consumer;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public interface ITagCamera {

    class VisionData {
        public Pose3d pose;
        public double timestamp;
        public Matrix<N3, N1> stdv;
    }

    public String getName();
    
    public void refreshEstimate(Consumer<VisionData> data);
}

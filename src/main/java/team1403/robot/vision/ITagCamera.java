package team1403.robot.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;

public interface ITagCamera {

    public String getName();

    public boolean hasPose();

    public Pose3d getPose();
   
    public double getTimestamp();

    public Matrix<N4, N1> getEstStdv();

    //make sure the estimate is not a bad estimate
    public boolean checkVisionResult();
}

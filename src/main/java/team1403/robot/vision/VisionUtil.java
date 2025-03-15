package team1403.robot.vision;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;

public class VisionUtil {

    //find camera offset through measurements
    public Transform3d findOffset(Pose3d current, Pose3d target) {
        return target.minus(current);
    }
    
}

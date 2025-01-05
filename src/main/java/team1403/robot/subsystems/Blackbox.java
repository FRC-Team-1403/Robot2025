package team1403.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Blackbox extends SubsystemBase {

    private static Pose2d target = null;
    private static Blackbox instance = null;

    private Blackbox() {}

    public static Blackbox getInstance() {

        if (instance == null) instance = new Blackbox();
        return instance;
    }
    
    public static Pose2d getTargetPosition() {
        return target;
    }

    public static boolean isValidTargetPosition() {
        return target != null;
    }

    @Override
    public void periodic() {
        //compute target position and other data here
    }
}

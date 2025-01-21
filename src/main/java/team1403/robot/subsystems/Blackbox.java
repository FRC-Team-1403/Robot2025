package team1403.robot.subsystems;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.lib.util.CougarUtil;

//WIP (work in progress)
public class Blackbox extends SubsystemBase {

    private static Pose2d target = null;
    private static Blackbox instance = null;

    enum ReefSelect {
        LEFT,
        RIGHT
    }

    private static Pose2d[] reefPosesLeftBLUE;
    private static Pose2d[] reefPosesRightBLUE;
    private static Pose2d[] reefPosesLeftRED;
    private static Pose2d[] reefPosesRightRED;
    private static ReefSelect reefSide = ReefSelect.LEFT;

    //meters
    private static final double kMaxAlignDist = 1.0;

    private Blackbox() {
        //12 different scoring locations on reef
        reefPosesLeftBLUE = new Pose2d[6];
        reefPosesRightBLUE = new Pose2d[6];
        reefPosesLeftRED = new Pose2d[6];
        reefPosesRightRED = new Pose2d[6];

        reefPosesLeftBLUE[0] = new Pose2d();
        reefPosesLeftBLUE[1] = new Pose2d();
        reefPosesLeftBLUE[2] = new Pose2d();
        reefPosesLeftBLUE[3] = new Pose2d();
        reefPosesLeftBLUE[4] = new Pose2d();
        reefPosesLeftBLUE[5] = new Pose2d();
        reefPosesRightBLUE[0] = new Pose2d();
        reefPosesRightBLUE[1] = new Pose2d();
        reefPosesRightBLUE[2] = new Pose2d();
        reefPosesRightBLUE[3] = new Pose2d();
        reefPosesRightBLUE[4] = new Pose2d();
        reefPosesRightBLUE[5] = new Pose2d();

        for(int i = 0; i < reefPosesLeftBLUE.length; i++) {
            reefPosesLeftRED[i] = FlippingUtil.flipFieldPose(reefPosesLeftBLUE[i]);
        }

        for(int i = 0; i < reefPosesRightBLUE.length; i++) {
            reefPosesRightRED[i] = FlippingUtil.flipFieldPose(reefPosesRightBLUE[i]);
        }

        //Manipulate red alliance positions here in case field elems move this year as well
    }

    public static Blackbox getInstance() {

        if (instance == null) instance = new Blackbox();
        return instance;
    }

    //TODO: bind this command to some button
    public static Command reefSelect(ReefSelect select) {
        return new InstantCommand(() -> reefSide = select);
    }

    private static Pose2d[] getReefPoses() {
        if(reefSide == ReefSelect.LEFT)
            return CougarUtil.getAlliance() == Alliance.Blue ? reefPosesLeftBLUE : reefPosesLeftRED;
        else
            return CougarUtil.getAlliance() == Alliance.Blue ? reefPosesRightBLUE : reefPosesRightRED;
    }
    
    public static Pose2d getNearestAlignPositionReef(Pose2d currentPose) {
        Pose2d nearest = CougarUtil.getNearest(currentPose, getReefPoses());
        if (nearest == null) return null;
        if (CougarUtil.getDistance(currentPose, nearest) > kMaxAlignDist) return null;

        return nearest;
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

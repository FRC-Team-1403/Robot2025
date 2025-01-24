package team1403.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.lib.util.CougarUtil;

//WIP (work in progress)
public class Blackbox extends SubsystemBase {

    private static Pose2d target = null;
    private static Blackbox instance = null;

    public enum ReefSelect {
        LEFT,
        RIGHT
    }

    private static Pose2d[] reefPosesLeftBLUE;
    private static Pose2d[] reefPosesRightBLUE;
    private static Pose2d[] reefPosesLeftRED;
    private static Pose2d[] reefPosesRightRED;
    private static ReefSelect reefSide = ReefSelect.LEFT;

    private static final double kHalfBumperLengthMeters = Units.inchesToMeters(26);

    //meters
    private static final double kMaxAlignDist = 2.5;

    private Blackbox() {
        //12 different scoring locations on reef
        reefPosesLeftBLUE = new Pose2d[6];
        reefPosesRightBLUE = new Pose2d[6];
        reefPosesLeftRED = new Pose2d[6];
        reefPosesRightRED = new Pose2d[6];

        reefPosesRightBLUE[0] = new Pose2d(4.212980794, 3.22745, Rotation2d.fromDegrees(-120));
        reefPosesRightBLUE[1] = new Pose2d(3.66, 3.8649, Rotation2d.fromDegrees(180));
        reefPosesRightBLUE[2] = new Pose2d(3.927019206, 4.66745, Rotation2d.fromDegrees(120));
        reefPosesRightBLUE[3] = new Pose2d(4.757019206, 4.83255, Rotation2d.fromDegrees(60));
        reefPosesRightBLUE[4] = new Pose2d(5.32, 4.1951, Rotation2d.kZero);
        reefPosesRightBLUE[5] = new Pose2d(5.042980794, 3.39255, Rotation2d.fromDegrees(-60));
        reefPosesLeftBLUE[0] = new Pose2d(3.927019206, 3.39255, Rotation2d.fromDegrees(-120));
        reefPosesLeftBLUE[1] = new Pose2d(3.66, 4.1951, Rotation2d.fromDegrees(180));
        reefPosesLeftBLUE[2] = new Pose2d(4.212980794, 4.83255, Rotation2d.fromDegrees(120));
        reefPosesLeftBLUE[3] = new Pose2d(5.042980794, 4.66745, Rotation2d.fromDegrees(60));
        reefPosesLeftBLUE[4] = new Pose2d(5.32, 3.8649, Rotation2d.fromDegrees(0));
        reefPosesLeftBLUE[5] = new Pose2d(4.757019206, 3.22745, Rotation2d.fromDegrees(-60));

        for(int i = 0; i < reefPosesLeftBLUE.length; i++) {
            reefPosesLeftBLUE[i] = CougarUtil.rotatePose2d(
                CougarUtil.addDistanceToPose(reefPosesLeftBLUE[i], kHalfBumperLengthMeters), 
                Rotation2d.k180deg);
            reefPosesLeftRED[i] = FlippingUtil.flipFieldPose(reefPosesLeftBLUE[i]);
        }

        for(int i = 0; i < reefPosesRightBLUE.length; i++) {
            reefPosesRightBLUE[i] = CougarUtil.rotatePose2d(
                CougarUtil.addDistanceToPose(reefPosesRightBLUE[i], kHalfBumperLengthMeters), 
                Rotation2d.k180deg);
            reefPosesRightRED[i] = FlippingUtil.flipFieldPose(reefPosesRightBLUE[i]);
        }

        //Manipulate red alliance positions here in case field elems move this year as well
    }

    public static Blackbox getInstance() {

        if (instance == null) instance = new Blackbox();
        return instance;
    }

    //TODO: bind this command to some button
    public static void reefSelect(ReefSelect select) {
        reefSide = select;
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

    @Override
    public void periodic() {
        //compute target position and other data here
        Logger.recordOutput("ReefPositions Blue Right", reefPosesRightBLUE);
        Logger.recordOutput("ReefPositions Blue Left", reefPosesLeftBLUE);
        Logger.recordOutput("ReefPositions Red Right", reefPosesRightRED);
        Logger.recordOutput("ReefPositions Red Left", reefPosesLeftRED);
    }
}

package team1403.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import team1403.lib.util.CougarUtil;
import team1403.robot.Constants;

//WIP (work in progress)
//Stores data that is shared between subsystems
public class Blackbox {

    public enum State{
        loading,
        driving,
        aligning,
        placing,
        ManualElevator
    }

    public enum ReefSelect {
        LEFT,
        RIGHT
    }

    public enum ReefScoreLevel {
        drive,
        L1,
        L2,
        L3, 
        L4
    }

    private static Pose2d[] reefPosesLeftBLUE;
    private static Pose2d[] reefPosesRightBLUE;
    private static Pose2d[] reefPosesLeftRED;
    private static Pose2d[] reefPosesRightRED;
    public static ReefSelect reefSide = ReefSelect.LEFT;
    public static ReefScoreLevel reefLevel = ReefScoreLevel.drive;
    private static boolean coralLoaded = false;
    private static boolean algaeLoaded = false;
    private static boolean aligning = false;
    private static boolean trigger = false;
    private static boolean manual = false;

    private static final double kHalfBumperLengthMeters = Units.inchesToMeters(25);

    public static State robotState = State.loading;

    //meters
    private static final double kMaxAlignDist = 2.5;

    private static final Alert debugModeAlert = 
        new Alert("Debug Mode Active, Expect Reduced Performance", AlertType.kWarning);
    private static final Alert sysIdActiveAlert =
        new Alert("SysID is enabled", AlertType.kInfo);

    public static void init() {
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
        reefPosesLeftBLUE[4] = new Pose2d(5.32, 3.8649, Rotation2d.kZero);
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

    public static void setManual(boolean man) {
        manual = man;
    }

    public static boolean isManual() {
        return manual;
    }
    public static void reefSelect(ReefSelect select) {
        reefSide = select;
    }

    public static void reefScoreLevel(ReefScoreLevel level) {
        reefLevel = level;
    }

    public static Command reefSelectCmd(ReefSelect select) {
        return new InstantCommand(() -> reefSelect(select));
    }

    public static Command reefScoreLevelCmd(ReefScoreLevel level) {
        return new InstantCommand(() -> reefScoreLevel(level));
    }

    private static Pose2d[] getReefPoses() {
        if(reefSide == ReefSelect.LEFT)
            return CougarUtil.getAlliance() == Alliance.Blue ? reefPosesLeftBLUE : reefPosesLeftRED;
        else
            return CougarUtil.getAlliance() == Alliance.Blue ? reefPosesRightBLUE : reefPosesRightRED;
    }

    public static void setAlgaeLoaded(boolean loaded) {
        algaeLoaded = loaded;
    }

    public static void setCoralLoaded(boolean loaded) {
        coralLoaded = loaded;
    }

    public static boolean isCoralLoaded() {
        return coralLoaded;
    }

    public static boolean isAlgaeLoaded() {
        return algaeLoaded;
    }

    public static void setAligning(boolean align) {
        aligning = align;
    }

    public static Command setAligningCmd(boolean align) {
        return new InstantCommand(() -> setAligning(align));
    }

    public static boolean isAligning() {
        return aligning;
    }

    public static void setTrigger(boolean trig) {
        trigger = trig;
    }

    public static boolean getTrigger() {
        return trigger;
    }

    public static boolean getCloseAlign(Pose2d pose) {
        Pose2d closest = getNearestAlignPositionReef(pose);
        if (closest == null) return false;
        if (CougarUtil.getDistance(pose, closest) > Constants.Vision.closeAlignDistance) return false;
        if (CougarUtil.dot(pose.getRotation(), closest.getRotation()) < 0.7) return false;
        return true;
    }

    private static final double kDotWeight = -0.5;
    private static double distanceHeuristic(Pose2d a, Pose2d b) {
        return CougarUtil.getDistance(a, b) + kDotWeight * CougarUtil.dot(a.getRotation(), b.getRotation());
    }

    public static Pose2d getNearestHeuristic(Pose2d a, Pose2d[] list) {
        if (list.length == 0) return null;
        Pose2d min = list[0];
        double min_dist = distanceHeuristic(a, list[0]);
        for(Pose2d b : list) {
            double dist = distanceHeuristic(a, b);
            if(dist < min_dist) {
                min_dist = dist;
                min = b;
            }
        }
        return min;
    }
    
    public static Pose2d getNearestAlignPositionReef(Pose2d currentPose) {
        Pose2d nearest = null;
        if (isCoralLoaded()) nearest = getNearestHeuristic(currentPose, getReefPoses());
        if (nearest == null) return null;
        if (CougarUtil.getDistance(currentPose, nearest) > kMaxAlignDist) return null;

        return nearest;
    }

    public static void periodic() {
        //compute target position and other data here
        Logger.recordOutput("ReefPositions Blue Right", reefPosesRightBLUE);
        Logger.recordOutput("ReefPositions Blue Left", reefPosesLeftBLUE);
        Logger.recordOutput("ReefPositions Red Right", reefPosesRightRED);
        Logger.recordOutput("ReefPositions Red Left", reefPosesLeftRED);

        debugModeAlert.set(Constants.DEBUG_MODE);
        sysIdActiveAlert.set(Constants.ENABLE_SYSID);
    }
}

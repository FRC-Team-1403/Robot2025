package team1403.lib.util;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class CougarUtil {
    
    public static Alliance getAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Blue);
    }

    public static boolean shouldMirrorPath() {
        return getAlliance() == Alliance.Red;
    }

    //overrides rotation of input pose2d with the one passed in
    public static Pose2d createPose2d(Pose2d pose, Rotation2d rot) {
        return new Pose2d(pose.getTranslation(), rot);
    }

    //rotates pose2d about itself :)
    public static Pose2d rotatePose2d(Pose2d pose, Rotation2d rot) {
        return new Pose2d(pose.getTranslation(), pose.getRotation().plus(rot));
    }

    public static Pose2d getInitialRobotPose() {
        if(getAlliance() == Alliance.Red)
            //FIXME: put a valid red alliance position
            return new Pose2d(new Translation2d(1 ,1), Rotation2d.k180deg);
        
        return new Pose2d(new Translation2d(1, 1), Rotation2d.kZero);
    }

    public static double getDistance(Pose2d a, Pose2d b) {
        return a.getTranslation().getDistance(b.getTranslation());
    }

    public static double dot(Rotation2d a, Rotation2d b) {
        return a.getCos() * b.getCos() + a.getSin() * b.getSin(); // 2d dot product: a_x * b_x + a_y * b_y
    }

    public static Pose2d addDistanceToPose(Pose2d pose, double distance) {
        return new Pose2d(pose.getTranslation().plus(
            new Translation2d(distance, pose.getRotation())), 
            pose.getRotation());
    }

    public static Pose2d addDistanceToPoseLeft(Pose2d pose, double distance) {
        return new Pose2d(pose.getTranslation().plus(
            new Translation2d(distance, pose.getRotation().plus(Rotation2d.kCCW_90deg))), 
            pose.getRotation());
    }

    //saves allocation comared to Pose2d.nearest
    public static Pose2d getNearest(Pose2d a, Pose2d[] list) {
        if (list.length == 0) return null;
        Pose2d min = list[0];
        double min_dist = getDistance(a, list[0]);
        for(Pose2d b : list) {
            double dist = getDistance(a, b);
            if(dist < min_dist) {
                min_dist = dist;
                min = b;
            }
        }
        return min;
    }

    private static final double kDotWeight = -0.5;
    public static Pose2d getNearestHeuristic(Pose2d a, Pose2d[] list) {
        if (list.length == 0) return null;
        Pose2d min = list[0];
        double min_dist = getDistance(a, list[0]) + kDotWeight * dot(a.getRotation(), list[0].getRotation());
        for(Pose2d b : list) {
            double dist = getDistance(a, b) + kDotWeight * dot(a.getRotation(), b.getRotation());
            if(dist < min_dist) {
                min_dist = dist;
                min = b;
            }
        }
        return min;
    }

    public static RobotConfig loadRobotConfig() {
        try {
            return RobotConfig.fromGUISettings();
        } catch (Exception e) {
            return null;
        }
    }

    public static DCMotorSim createDCMotorSim(DCMotor motor, double gearing, double MOI) {
        return new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, MOI, gearing), motor);
    }
}

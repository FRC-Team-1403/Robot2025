package team1403.lib.util;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import team1403.robot.Constants;

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

    public static Pose2d getInitialRobotPose() {
        if(getAlliance() == Alliance.Red)
            //FIXME: put a valid red alliance position
            return new Pose2d(new Translation2d(1 ,1), Rotation2d.k180deg);
        
        return new Pose2d(new Translation2d(1, 1), Rotation2d.kZero);
    }

    //TODO: update when we get robot
    private static RobotConfig config = new RobotConfig(
        Pounds.of(120), 
        KilogramSquareMeters.of(1),
        new ModuleConfig(
            Constants.Swerve.kWheelDiameterMeters / 2., 
            Constants.Swerve.kMaxSpeed, 
            1.4, 
            DCMotor.getNEO(1).withReduction(1.0/Constants.Swerve.kDriveReduction), 
            Constants.Swerve.kDriveCurrentLimit, 
            1), 
        Constants.Swerve.kModulePositions);

    public static RobotConfig loadRobotConfig() {
        return config;
    }

    public static DCMotorSim createDCMotorSim(DCMotor motor, double gearing, double MOI) {
        return new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, MOI, gearing), motor);
    }
}

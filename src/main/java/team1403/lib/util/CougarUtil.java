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

    //overrides rotation of input pose3d with the one passed in
    public static Pose3d createPose3d(Pose3d pose, Rotation3d rot) {
        return new Pose3d(pose.getTranslation(), rot);
    }

    public static Pose3d getInitialRobotPose() {
        if(getAlliance() == Alliance.Red)
            return new Pose3d(new Translation3d(1 ,1, 0), new Rotation3d(0, 0, Math.PI));
        
        return new Pose3d(new Translation3d(1, 1, 0), Rotation3d.kZero);
    }

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

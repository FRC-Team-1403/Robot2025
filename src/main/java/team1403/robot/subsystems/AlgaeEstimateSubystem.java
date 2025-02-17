package team1403.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.robot.Constants;
import team1403.robot.vision.LimelightHelpers;
import team1403.robot.vision.LimelightHelpers.LimelightResults;


public class AlgaeEstimateSubystem extends SubsystemBase {
    double fovX = 62.544;
    double fovY = 48.953;
    private final double k = Constants.Vision.algaeEstimateKonstant;
    double distance = 1;
    private double limelightOldY = getY(LimelightHelpers.getTA("limelight"));
    private double limelightOldDistance = getDistance(LimelightHelpers.getTA("limelight"));
    private double OldArea = LimelightHelpers.getTA("limelight");


    public double getDistance(double Area) {
        double a = Area;
        return k*(Math.sqrt(1.0/a));
    }

    public double getX(double Area) {
        return Math.tan(LimelightHelpers.getTX("limelight") * Math.PI/180) * getDistance(Area);
    }

    public double getZ(double Area) {
        return -Math.tan(LimelightHelpers.getTY("limelight") * Math.PI/180) * getDistance(Area);
    }

    public double getY(double Area) {
        /*return getDistance()
            * Math.cos(LimelightHelpers.getTX("limelight") * Math.PI/180) 
            * Math.cos(LimelightHelpers.getTY("limelight") * Math.PI/180);*/
        return getDistance(Area);
    }

    public Pose3d getPose(double Area) {
        if(LimelightHelpers.getTV("limelight")) {
            //LimelightResults res = LimelightHelpers.getLatestResults("limelight");
            //Logger.recordOutput("test", res.targets_Detector[0].pts);
            Transform3d output = new Transform3d(
                new Translation3d(
                getX(Area),
                getY(Area),
                getZ(Area)),
                Rotation3d.kZero
            );

            output = output.plus(Constants.Swerve.kLimelightTransform);
            return new Pose3d(new Pose2d(3, 3, Rotation2d.kZero)).transformBy(output);
        }
        return null;
    }

    public Pose3d fixPose() {

        double limelightCurrentY = getY(LimelightHelpers.getTA("limelight"));
        double limelightCurrentDistance = getDistance(LimelightHelpers.getTA("limelight"));
        double currentArea = LimelightHelpers.getTA("limelight");
    
        if (limelightOldY != limelightCurrentY) {
            limelightOldY = limelightCurrentY;
            return getPose(currentArea);
        } else if (limelightCurrentY == limelightOldY && limelightOldDistance - limelightCurrentDistance >= 0.5 && OldArea != currentArea) {
            if (OldArea > currentArea) {
                return getPose(OldArea);
            } else {
                OldArea = currentArea;
                return getPose(currentArea);
            }
        } else {
            return getPose(currentArea);
        }
    }

    public void periodic() {
        
        if(LimelightHelpers.getTV("limelight"))
        {
            Logger.recordOutput("dist (m)", getDistance(LimelightHelpers.getTA("limelight")));
            Logger.recordOutput("X distance", getX(LimelightHelpers.getTA("limelight")));
            Logger.recordOutput("Y distance", getZ(LimelightHelpers.getTA("limelight")));
            Logger.recordOutput("pose", getPose(LimelightHelpers.getTA("limelight")));
        }
    }
}
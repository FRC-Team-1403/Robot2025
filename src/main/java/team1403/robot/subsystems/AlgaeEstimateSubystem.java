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


    public double getDistance() {
        double a = LimelightHelpers.getTA("limelight");
        return k*(Math.sqrt(1.0/a));
    }

    public double getX() {
        return Math.tan(LimelightHelpers.getTX("limelight") * Math.PI/180) * getDistance();
    }

    public double getZ() {
        return -Math.tan(LimelightHelpers.getTY("limelight") * Math.PI/180) * getDistance();
    }

    public double getY() {
        /*return getDistance()
            * Math.cos(LimelightHelpers.getTX("limelight") * Math.PI/180) 
            * Math.cos(LimelightHelpers.getTY("limelight") * Math.PI/180);*/
        return getDistance();
    }

    public Pose3d getPose() {
        if(LimelightHelpers.getTV("limelight")) {
            LimelightResults res = LimelightHelpers.getLatestResults("limelight");
            Logger.recordOutput("test", res.targets_Detector[0].pts);
            Pose3d output = new Pose3d(
                new Translation3d(
                getX(),
                getY(),
                getZ()),
                Rotation3d.kZero
            );

            output.transformBy(Constants.Swerve.kCameraTransfrom.inverse());
            output.transformBy(new Transform3d(new Transform2d(16,16, Rotation2d.kZero)));
            return output;
        }
        return null;
    }

    public void periodic() {
        if(LimelightHelpers.getTV("limelight"))
        {
            Logger.recordOutput("dist (m)", getDistance());
            Logger.recordOutput("X distance", getX());
            Logger.recordOutput("Y distance", getZ());
            Logger.recordOutput("pose", getPose());
        }
    }
}
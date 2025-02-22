package team1403.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.robot.Constants;
import team1403.robot.vision.LimelightHelpers;

public class AlgaeEstimateSubystem extends SubsystemBase {
    double fovX = 62.544;
    double fovY = 48.953;
    private final double k = Constants.Vision.algaeEstimateKonstant;

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

    public boolean isDisregard() {

        boolean disregard = false;
        double[] corners = LimelightHelpers.getCorners("limelight");

        for(int i = 0; i < corners.length; i++) {
            if(i % 2 == 0) {
                if(corners[i] > 1230 || corners[i] < 50) {
                    disregard = true;
                }
            }
            else {
                if(corners[i] > 910 || corners[i] < 50) {
                    disregard = true;
                }
            }
        }

    return disregard;
    }

    public Pose3d getPose() {

        if(LimelightHelpers.getTV("limelight") && isDisregard() == false) {
            //Logger.recordOutput("test", res.targets_Detector[0].pts);
            Transform3d output = new Transform3d(
                new Translation3d(
                getX(), 
                getY(),
                getZ()),
                Rotation3d.kZero
            );

            output = output.plus(Constants.Swerve.kLimelightTransform);
            return new Pose3d(new Pose2d(0, 0, Rotation2d.kZero)).transformBy(output);
        }
        return null;
    }

    public void periodic() {

        

        // if (atEdge() && OldArea > currentArea) {
        //     getPose(OldArea);
        // } else if (atEdge() && currentArea > OldArea) {
        //     getPose(currentArea);
        // } else if (OldArea > currentArea) {
        //     getPose(OldArea);
        // } else if (currentArea > OldArea) {
        //     getPose(currentArea);
        // }

        if(LimelightHelpers.getTV("limelight"))
        {
            Logger.recordOutput("dist (m)", getDistance());
            Logger.recordOutput("X distance", getX());
            Logger.recordOutput("Y distance", getZ());
            Logger.recordOutput("pose", getPose());
            Logger.recordOutput("corners", LimelightHelpers.getCorners("limelight"));
            Logger.recordOutput("disregard", isDisregard());
        }
    }
}
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
    private Pose3d value;


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

    public boolean atEdge() {

        boolean value = false;
        double[] corners = LimelightHelpers.getCorners("limelight");

        for (int index = 0; index < corners.length; index++) {   
            if (index % 2 == 0) {
                if (corners[index] > 1230 || corners[index] < 50) {
                    value = true;
                }
            } else {
                if (corners[index] > 910 || corners[index] < 50) {
                    value = true;
                }
            }
        }

        return value;

    }

    public Pose3d properPose() {

        double limelightCurrentY = getY(LimelightHelpers.getTA("limelight"));
        double limelightCurrentDistance = getDistance(LimelightHelpers.getTA("limelight"));
        double currentArea = LimelightHelpers.getTA("limelight");

        if (limelightCurrentDistance != limelightOldDistance && limelightCurrentY - limelightOldY <= 0.9) {
            if (atEdge()) {
                value = getPose(OldArea);
            }
            if (!atEdge()) {
                OldArea = currentArea;
                limelightOldDistance = limelightCurrentDistance;
            }
        } else {
            value = getPose(currentArea);
        }

        return value;

    }

    public void periodic() {
        
        
        System.out.println(OldArea);
        System.out.println(atEdge());
        System.out.println(atEdge());

        

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
            Logger.recordOutput("dist (m)", getDistance(LimelightHelpers.getTA("limelight")));
            Logger.recordOutput("X distance", getX(LimelightHelpers.getTA("limelight")));
            Logger.recordOutput("Y distance", getZ(LimelightHelpers.getTA("limelight")));
            Logger.recordOutput("pose", properPose());

            Logger.recordOutput("OldArea", OldArea);
            Logger.recordOutput("limelightOldY", limelightOldY);
            Logger.recordOutput("limelightOldDistance", limelightOldDistance);
            // Logger.recordOutput("currentArea", currentArea);
            // Logger.recordOutput("limelightCurrentY", limelightCurrentY);
            // Logger.recordOutput("limelightCurrentDistance", limelightCurrentDistance);

        }
    }
}
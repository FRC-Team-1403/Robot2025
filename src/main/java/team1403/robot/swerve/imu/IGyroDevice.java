package team1403.robot.swerve.imu;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

public interface IGyroDevice {
    
    public Rotation2d getRotation2d();

    public Rotation3d getRotation3d();

    public double getAngularVelocity();
    
    //reset gyro z-axis heading to 0
    public void reset();
}

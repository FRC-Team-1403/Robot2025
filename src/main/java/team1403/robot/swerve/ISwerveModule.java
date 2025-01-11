package team1403.robot.swerve;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface ISwerveModule {

    enum DriveControlType {
        Velocity,
        Voltage
    }

    enum SteerControlType {
        Angle,
        Voltage
    }

    public SwerveModuleState getState();

    public SwerveModulePosition getModulePosition();

    public void set(DriveControlType type, double value, SteerControlType s_type, double steerAngle);
}

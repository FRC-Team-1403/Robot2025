package team1403.robot.swerve.module;

import com.pathplanner.lib.util.DriveFeedforwards;

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

    enum ModuleFeedforwardType {
        None,
        XYForce,
    }

    public SwerveModuleState getState();

    public SwerveModulePosition getModulePosition();

    public void set(DriveControlType type, double driveValue, SteerControlType s_type, double steerValue, DriveFeedforwards ff, int index);

    public default void set(DriveControlType type, double driveValue, SteerControlType s_type, double steerValue) {
        set(type, driveValue, s_type, steerValue, null, -1);
    }
}

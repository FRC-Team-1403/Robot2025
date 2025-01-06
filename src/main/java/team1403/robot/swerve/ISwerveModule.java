package team1403.robot.swerve;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface ISwerveModule {

    enum ModControlType {
        Velocity,
        Voltage
    }

    public SwerveModuleState getState();

    public SwerveModulePosition getModulePosition();

    public void set(ModControlType type, double value, double steerAngle);
}

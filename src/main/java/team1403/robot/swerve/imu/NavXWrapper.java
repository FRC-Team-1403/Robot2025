package team1403.robot.swerve.imu;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS.NavXUpdateRate;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NavXWrapper implements IGyroDevice {
    
    private final AHRS m_navx;

    public NavXWrapper(NavXComType type, NavXUpdateRate rate) {
        m_navx = new AHRS(type, rate);

        //wait for navx to finish calibrating before finishing construction
        if (m_navx.isConnected())
            while (m_navx.isCalibrating());

        //add gyro to smartdashboard
        SmartDashboard.putData("Gyro", m_navx);
    }

    @Override
    public Rotation2d getRotation2d() {
        return m_navx.getRotation2d();
    }

    @Override
    public Rotation3d getRotation3d() {
        return m_navx.getRotation3d();
    }

    @Override
    public double getAngularVelocity() {
        return m_navx.getRate();
    }

    @Override
    public void reset() {
        m_navx.reset();
    }
}

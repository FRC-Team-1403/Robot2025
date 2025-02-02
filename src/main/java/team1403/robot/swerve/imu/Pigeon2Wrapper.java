package team1403.robot.swerve.imu;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Pigeon2Wrapper implements IGyroDevice {

    private final Pigeon2 m_pigeon;

    public Pigeon2Wrapper(int id, String canbus) {
        m_pigeon = new Pigeon2(id, canbus);

        Pigeon2Configuration config = new Pigeon2Configuration();

        //add custom configuration here if nessessary

        m_pigeon.getConfigurator().apply(config);

        SmartDashboard.putData("Gyro", m_pigeon);
    }

    public Pigeon2Wrapper(int id) {
        this(id, ""); //uses default canbus
    }

    @Override
    public boolean isConnected() {
        return m_pigeon.isConnected();
    }

    @Override
    public Rotation2d getRotation2d() {
        return m_pigeon.getRotation2d();
    }

    @Override
    public Rotation3d getRotation3d() {
        return m_pigeon.getRotation3d();
    }

    @Override
    public double getAngularVelocity() {
        //CW+ instead of CCW+, matches navx which our code is based on
        return -m_pigeon.getAngularVelocityZWorld(true).getValue().in(DegreesPerSecond);
    }

    @Override
    public void reset() {
        m_pigeon.reset();
    }
    
}

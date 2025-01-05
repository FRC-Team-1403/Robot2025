package team1403.robot.swerve;

import java.util.Arrays;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import team1403.robot.Constants;

public class SyncSwerveDrivePoseEstimator {

    private final SwerveDrivePoseEstimator3d m_odometer;
    private final Supplier<SwerveModulePosition[]> m_modPosSup;
    private final Supplier<Rotation3d> m_gyroRotation;
    private final AtomicReference<Pose3d> m_cachedPose;
    private final AtomicInteger m_updateCount;


    public SyncSwerveDrivePoseEstimator(Pose3d initialPose, Supplier<Rotation3d> gyroRotation, Supplier<SwerveModulePosition[]> modulePoses) {
        m_modPosSup = modulePoses;
        m_gyroRotation = gyroRotation;
        m_cachedPose = new AtomicReference<>(initialPose);
        m_odometer = new SwerveDrivePoseEstimator3d(Constants.Swerve.kDriveKinematics, m_gyroRotation.get(), m_modPosSup.get(), initialPose);
        m_updateCount = new AtomicInteger(0);
    }

    public int resetUpdateCount() {
        return m_updateCount.getAndSet(0);
    }

    private void updateCachedPose() {
        m_cachedPose.set(m_odometer.getEstimatedPosition());
    }

    public void update() {
        synchronized(this) {
            m_cachedPose.set(m_odometer.update(m_gyroRotation.get(), m_modPosSup.get()));
        }
        m_updateCount.incrementAndGet();
    }

    public void addVisionMeasurement(Pose3d measurement, double timestamp, Matrix<N4, N1> stdev) {
        synchronized(this) {
            m_odometer.addVisionMeasurement(measurement, timestamp, stdev);
            updateCachedPose();
        }
    }

    public void addVisionMeasurement(Pose3d measurement, double timestamp) {
        synchronized(this) {
            m_odometer.addVisionMeasurement(measurement, timestamp);
            updateCachedPose();
        }
    }

    public void resetPosition(Pose3d pose) {
        synchronized(this) {
            m_odometer.resetPosition(m_gyroRotation.get(), m_modPosSup.get(), pose);
            m_cachedPose.set(pose);
        }
    }

    public final Pose3d getPose() {
        return m_cachedPose.get();
    }

    
}

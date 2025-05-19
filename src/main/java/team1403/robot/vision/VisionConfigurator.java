package team1403.robot.vision;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionConfigurator {
    private String m_name;
    private Supplier<Transform3d> m_cameraTransform = () -> Transform3d.kZero;
    private Supplier<Pose2d> m_robotPose = null;
    private DoubleSupplier m_poseTimestamp = null;

    /* Yaw rate CCW+, rad/s */
    private DoubleSupplier m_yawRate = () -> 0.0;

    // Default deviations for normal april tag detection
    private Matrix<N3, N1> m_deviations = VecBuilder.fill(2, 2, 3);

    // Deviations for trig solve (make sure to set third element to infinity)
    private Matrix<N3, N1> m_deviationsTrig = VecBuilder.fill(1, 1, Double.POSITIVE_INFINITY);

    // Should we even do trig solve ?
    // Requirements: Set Robot Rotation Supplier and Trig Deviations
    private boolean m_enableTrigSolve = false;

    public VisionConfigurator() {}

    /* Configuration chaining methods */

    // In case we want to chnage name after init (for a second camera perhaps)
    public VisionConfigurator withName(String name)
    {
        m_name = name;
        return this;
    }
    
    public VisionConfigurator withTrigSolve(boolean enabled)
    {
        m_enableTrigSolve = enabled;
        return this;
    }

    public VisionConfigurator withDeviations(Matrix<N3, N1> dev)
    {
        m_deviations = dev;
        return this;
    }

    public VisionConfigurator withDeviationsTrig(Matrix<N3, N1> trigDev)
    {
        m_deviationsTrig = trigDev;
        return this;
    }

    public VisionConfigurator withTransform(Supplier<Transform3d> transform)
    {
        m_cameraTransform = transform;
        return this;
    }

    public VisionConfigurator withRobotPose(Supplier<Pose2d> pose, DoubleSupplier pose_timestamp)
    {
        m_robotPose = pose;
        m_poseTimestamp = pose_timestamp;
        return this;
    }

    public VisionConfigurator withYawRate(DoubleSupplier rate)
    {
        m_yawRate = rate;
        return this;
    }

    /* Getter methods */

    public boolean getTrigSolveEnabled() {
        return m_enableTrigSolve && m_robotPose != null && m_poseTimestamp != null;
    }

    public String getName() {
        return m_name;
    }

    public Supplier<Transform3d> getTransform3d() {
        return m_cameraTransform;
    }

    public Supplier<Pose2d> getRobotPose() {
        return m_robotPose;
    }

    public DoubleSupplier getPoseTimestamp() {
        return m_poseTimestamp;
    }

    public DoubleSupplier getYawRate() {
        return m_yawRate;
    }

    public Matrix<N3, N1> getDeviations() {
        return m_deviations;
    }

    public Matrix<N3, N1> getDeviationsTrig() {
        return m_deviationsTrig;
    }
}

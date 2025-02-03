package team1403.robot.swerve.module;

import java.util.concurrent.locks.ReentrantLock;

import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.lib.util.CougarUtil;
import team1403.robot.Constants;

public class SimSwerveModule extends SubsystemBase implements ISwerveModule {

    private final String m_name;
    private final SwerveModuleState m_state;
    private final SwerveModulePosition m_position;
    private final DCMotorSim m_driveSim;
    private final DCMotorSim m_steerSim;
    private final PIDController m_steerController = new PIDController(6, 0, 0);
    private final PIDController m_driveController = new PIDController(0.04, 0, 0);
    private final SimpleMotorFeedforward m_driveFF = new SimpleMotorFeedforward(0, 12/Constants.Swerve.kMaxSpeed, 0.1);

    private static final double PID_PERIOD = 0.004;
    private final Notifier m_pidNotifier;
    private final ReentrantLock m_lock;


    public SimSwerveModule(String name) {
        m_name = name;
        m_position = new SwerveModulePosition();
        m_state = new SwerveModuleState();
        m_driveSim = CougarUtil.createDCMotorSim(DCMotor.getNEO(1), 1/Constants.Swerve.kDriveReduction, 0.025);
        m_steerSim = CougarUtil.createDCMotorSim(DCMotor.getNEO(1), 1/Constants.Swerve.kSteerReduction, 0.004);

        m_steerController.enableContinuousInput(-Math.PI, Math.PI);

        m_lock = new ReentrantLock();

        m_pidNotifier = new Notifier(this::simLoop);
        m_pidNotifier.setName("SimSwervePID " + name);
        m_pidNotifier.startPeriodic(PID_PERIOD);
    }

    @Override
    public String getName() {
        return m_name;
    }

    private double getDriveVelocity() {
        return m_driveSim.getAngularVelocityRadPerSec() * Constants.Swerve.kWheelRadiusMeters; //omega * r = v
    }

    private double getDrivePosition() {
        return m_driveSim.getAngularPositionRad() * Constants.Swerve.kWheelRadiusMeters;
    }

    @Override
    public SwerveModuleState getState() {
        m_state.angle = Rotation2d.fromRadians(m_steerSim.getAngularPositionRad());
        m_state.speedMetersPerSecond = getDriveVelocity();

        return m_state;
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        m_position.angle = Rotation2d.fromRadians(m_steerSim.getAngularPositionRad());
        m_position.distanceMeters = getDrivePosition();

        return m_position;
    }

    @Override
    public void set(DriveControlType type, double driveValue, SteerControlType s_type, double steerValue, DriveFeedforwards ff, int index) {
        m_lock.lock();
        if (s_type == SteerControlType.Angle) {
            m_steerController.setSetpoint(MathUtil.angleModulus(steerValue));
        }
        if (type == DriveControlType.Velocity) {
            m_driveController.setSetpoint(driveValue);
        }
        m_lock.unlock();
    }

    public void simLoop() {
        m_lock.lock();
        double driveVel = getDriveVelocity(); 
        double driveVolt = m_driveController.calculate(driveVel) + 
            m_driveFF.calculateWithVelocities(driveVel, m_driveController.getSetpoint());

        m_driveSim.setInputVoltage(driveVolt);
        m_steerSim.setInputVoltage(m_steerController.calculate(m_steerSim.getAngularPositionRad()));
        m_lock.unlock();

        m_driveSim.update(PID_PERIOD);
        m_steerSim.update(PID_PERIOD);
    }
    
}

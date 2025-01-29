package team1403.robot.swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.robot.Constants;
import team1403.robot.Constants.Swerve;

import static edu.wpi.first.units.Units.Radians;
import org.littletonrobotics.junction.Logger;

/**
 * Represents a swerve module. Consists of a drive motor, steer motor, 
 * and their respective relative encoders.
 * Also consists of a absolute encoder to track steer angle.
 */
public class SwerveModule extends SubsystemBase implements ISwerveModule {
    private final SparkMax m_driveMotor;
    private final SparkMax m_steerMotor;

    private final CANcoder m_absoluteEncoder;
    private StatusSignal<Angle> m_positionSignal;
    private final double m_absoluteEncoderOffset;
    private final RelativeEncoder m_driveRelativeEncoder;
    private final RelativeEncoder m_steerRelativeEncoder;
    private final SparkClosedLoopController m_drivePIDController;
    private final SparkClosedLoopController m_steerPIDController;
    private final String m_name;
    private final boolean m_inverted;
    private double m_lastVelocitySetpoint;

    private final SwerveModuleState m_moduleState = new SwerveModuleState();
    private final SwerveModulePosition m_modulePosition = new SwerveModulePosition();
    private final SimpleMotorFeedforward m_driveFeedforward 
                              = new SimpleMotorFeedforward(
                                Constants.Swerve.kSDrive, 
                                Constants.Swerve.kVDrive,
                                Constants.Swerve.kADrive); 


    /**
     * Swerve Module represents a singular swerve module for a
     * swerve drive train.
     *
     * <p>Each swerve module consists of a drive motor,
     * changing the velocity of the wheel, and a steer motor, changing
     * the angle of the actual wheel inside of the module.
     *
     * <p>The swerve module also features
     * an absolute encoder to ensure the angle of
     * the module is always known, regardless if the bot is turned off
     * or not.
     *
     */
    public SwerveModule(String name, int driveMotorPort, int steerMotorPort,
        int canCoderPort, double offset) {
      this(name, driveMotorPort, steerMotorPort, canCoderPort, offset, true);
    }

    public SwerveModule(String name, int driveMotorPort, int steerMotorPort,
    int canCoderPort, double offset, boolean inverted)
    {
      m_inverted = inverted;
      m_name = name;
      m_lastVelocitySetpoint = 0;

      m_driveMotor = new SparkMax(driveMotorPort, MotorType.kBrushless);
      m_steerMotor = new SparkMax(steerMotorPort, MotorType.kBrushless);
      m_absoluteEncoder = new CANcoder(canCoderPort);
      m_driveRelativeEncoder = m_driveMotor.getEncoder();
      m_steerRelativeEncoder = m_steerMotor.getEncoder();
      m_absoluteEncoderOffset = offset;
      m_drivePIDController = m_driveMotor.getClosedLoopController();
      m_steerPIDController = m_steerMotor.getClosedLoopController();

      initSteerMotor();
      initDriveMotor();
      initEncoders();

      m_driveMotor.setCANTimeout(0);
      m_steerMotor.setCANTimeout(0);
    }

    @Override
    public String getName() {
      return m_name;
    }

    private void initEncoders() {
      // Config absolute encoder
      if (m_absoluteEncoder.getMagnetHealth().getValue() != MagnetHealthValue.Magnet_Green) {
        System.err.println("CANCoder magnetic field strength is unacceptable.");
      }
      MagnetSensorConfigs magnetSensor = new MagnetSensorConfigs();
      //in units of rotations
      magnetSensor.MagnetOffset = Units.radiansToRotations(MathUtil.angleModulus(m_absoluteEncoderOffset));
      magnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
      magnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

      CANcoderConfiguration config = new CANcoderConfiguration().withMagnetSensor(magnetSensor);

      m_absoluteEncoder.getConfigurator().apply(config, 0.250);
      m_positionSignal = m_absoluteEncoder.getAbsolutePosition();
      m_positionSignal.setUpdateFrequency(Constants.Swerve.kModuleUpdateRateHz);
      m_absoluteEncoder.optimizeBusUtilization();

      //avoid overrun, and get more up to date values for PID
      //m_absoluteEncoder.getPosition().setUpdateFrequency(500, 0.002);

      //m_absoluteEncoder.setPositionToAbsolute();
      //m_absoluteEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10, 250);

      // YAGSL website includes conversion factors for MK4 L3 drive, so instead of calcluating, 
      // we are are using their number
      // double drivePositionConversionFactor = Math.PI * Swerve.kWheelDiameterMeters 
      //       * Swerve.kDriveReduction (gear ratios);

      m_steerRelativeEncoder.setPosition(getAbsoluteAngle());
    }

    private void initSteerMotor() {
      SparkMaxConfig config = new SparkMaxConfig();

      config
        .idleMode(IdleMode.kBrake)
        .inverted(false)
        .voltageCompensation(Swerve.kVoltageSaturation)
        .smartCurrentLimit(Swerve.kSteerCurrentLimit);
      config.closedLoop
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(-Math.PI, Math.PI)
        .pid(Swerve.kPTurning, Swerve.kITurning, Swerve.kDTurning)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .outputRange(-1, 1);
      config.encoder
        .positionConversionFactor(Constants.Swerve.kSteerPositionConversionFactor)
        .velocityConversionFactor(Constants.Swerve.kSteerPositionConversionFactor / 60.);
      config.signals
        .primaryEncoderPositionPeriodMs(Swerve.kModuleUpdateRateMs);

      m_steerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void initDriveMotor() {
      SparkMaxConfig config = new SparkMaxConfig();

      config
        .idleMode(IdleMode.kBrake)
        .closedLoopRampRate(0)
        .openLoopRampRate(0)
        .voltageCompensation(Constants.Swerve.kVoltageSaturation)
        .smartCurrentLimit(Constants.Swerve.kDriveCurrentLimit)
        .inverted(m_inverted);
      config.closedLoop
        .pid(Constants.Swerve.kPDrive, Constants.Swerve.kIDrive, Constants.Swerve.kDDrive)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .outputRange(-1, 1);
      config.encoder
        //.uvwAverageDepth(2)
        //.uvwMeasurementPeriod(10)
        .positionConversionFactor(Constants.Swerve.kDrivePositionConversionFactor)
        .velocityConversionFactor(Constants.Swerve.kDrivePositionConversionFactor / 60.);
      config.signals
        .primaryEncoderVelocityPeriodMs(Constants.Swerve.kModuleUpdateRateMs)
        .primaryEncoderPositionPeriodMs(Constants.Swerve.kModuleUpdateRateMs);

      m_driveMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      // //slot 0 is used by default
      // m_drivePIDController.setSmartMotionMaxVelocity(6000, 0);
      // m_drivePIDController.setSmartMotionMinOutputVelocity(0, 0);
      // m_drivePIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);

      // m_driveMotor.setIdleMode(IdleMode.kCoast);
    }

    /**
     * Method for setting the drive voltage and steering angle.
     *
     * @param driveMetersPerSecond driving meters per second.
     * @param steerValue           steering angle.
     *
     */
    public void set(DriveControlType type, double driveValue, SteerControlType s_type, double steerValue, DriveFeedforwards ff, int index) {
      // Set driveMotor according to velocity input
      // System.out.println("drive input speed: " + driveMetersPerSecond);

      double absAngle = getAbsoluteAngle();
      //get the angle error between steer rel enc and abs enc
      double relativeErr = Math.abs(MathUtil.angleModulus(getSteerRotation() - absAngle));
      //rad/s
      double steerVel = m_steerRelativeEncoder.getVelocity();
      
      //if we dynamically correct while rotating the PID will get angry, error is also higher when in motion, since values aren't time synced
      if(relativeErr > Units.degreesToRadians(15) && Math.abs(steerVel) < 0.1) {
        System.out.println(getName() + " Encoder Reset!");
        m_steerRelativeEncoder.setPosition(absAngle);
      }

      // Set steerMotor according to position of encoder
      if(s_type == SteerControlType.Angle)
        m_steerPIDController.setReference(steerValue, ControlType.kPosition);
      else if(s_type == SteerControlType.Voltage)
        m_steerPIDController.setReference(steerValue, ControlType.kVoltage);

      if(type == DriveControlType.Velocity) {
        driveValue *= MathUtil.clamp(Math.cos(steerValue - absAngle), 0, 1);
        driveValue += steerVel * Constants.Swerve.kCouplingRatio;
        driveValue = MathUtil.clamp(driveValue, -Constants.Swerve.kMaxSpeed, Constants.Swerve.kMaxSpeed);

        m_drivePIDController.setReference(driveValue, ControlType.kVelocity, ClosedLoopSlot.kSlot0,
                      MathUtil.clamp(m_driveFeedforward.calculateWithVelocities(m_lastVelocitySetpoint, driveValue), -12, 12));
        m_lastVelocitySetpoint = driveValue;
      } else if (type == DriveControlType.Voltage) {
        m_drivePIDController.setReference(driveValue, ControlType.kVoltage);
        //better than it being completely wrong
        m_lastVelocitySetpoint = getDriveVelocity();
      }

      Logger.recordOutput(getName() + "/EncError", relativeErr);
    }

    /**
     * Gets the current angle reading of the encoder in radians.
     *
     * @return The current angle in radians. Range: [-pi, pi)
     */
    private synchronized double getAbsoluteAngle() {
      return MathUtil.angleModulus(m_positionSignal.refresh().getValue().in(Radians));
    }

    /**
     * Gets current drive encoder position
     * @return The current (coupling compensated) position of the drive encoder
     */
    private synchronized double getDrivePosition() {
      return m_driveRelativeEncoder.getPosition() - getSteerPosition() * Constants.Swerve.kCouplingRatio;
    }

    /**
     * Gets the current steer encoder angle
     * @return the current angle of the steer relative encoder. Range: [-pi, pi)
     */
    private double getSteerRotation() {
      return MathUtil.angleModulus(getSteerPosition());
    }

    /**
     * Gets the current steer encoder angle
     * @return the current angle of the steer relative encoder. Range: [-inf, inf)
     */
    private double getSteerPosition() {
      return m_steerRelativeEncoder.getPosition();
    }
  
    /**
     * Returns the SwerveModulePosition of this particular module.
     *
     * @return the SwerveModulePosition, which represents the distance 
     *         travelled and the angle of the module.
     */
    public synchronized SwerveModulePosition getModulePosition() {
      m_modulePosition.angle = Rotation2d.fromRadians(getAbsoluteAngle());
      m_modulePosition.distanceMeters = getDrivePosition();

      return m_modulePosition;
    }

    /**
     * Returns the current velocity of the drive motor.
     *
     * @return the current velocity of the drive motor
     */
    private double getDriveVelocity() {
      return m_driveRelativeEncoder.getVelocity();
    }

    /**
     * Returns the current state of the swerve module as defined by 
     * the relative encoders of the drive and steer motors.
     *
     * @return the current state of the swerve module
     */
    public SwerveModuleState getState() {
      m_moduleState.angle = Rotation2d.fromRadians(getAbsoluteAngle());
      m_moduleState.speedMetersPerSecond = getDriveVelocity();

      return m_moduleState;
    }

    @Override
    public void periodic() {
      //Logger.recordOutput(m_name + "/Drive Current", m_driveMotor.getOutputCurrent());
      //Logger.recordOutput(m_name + "/Steer Current", m_steerMotor.getOutputCurrent());
    }
  }
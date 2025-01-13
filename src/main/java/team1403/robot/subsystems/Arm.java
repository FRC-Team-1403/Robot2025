package team1403.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.robot.Constants;

public class Arm extends SubsystemBase {
  // lead motor
  private final SparkMax m_leftMotor;
  // following motor
  private final SparkMax m_rightMotor;
  private final DutyCycleEncoder m_armEncoder;
  private final ProfiledPIDController m_armPid;
  private final ArmFeedforward m_feedforward;

  // Setpoints
  private double m_pivotAngleSetpoint;

  private Mechanism2d m_mechanism;
  private MechanismRoot2d m_mechanismRoot;
  private MechanismLigament2d m_armMech;

  public Arm() {
    m_feedforward = new ArmFeedforward(0, Constants.Arm.kFeedforwardG, Constants.Arm.kFeedforwardV);
    m_leftMotor = new SparkMax(Constants.CanBus.leftPivotMotorID, MotorType.kBrushless);
    m_rightMotor = new SparkMax(Constants.CanBus.rightPivotMotorID, MotorType.kBrushless);
    m_armEncoder = new DutyCycleEncoder(Constants.RioPorts.kArmAbsoluteEncoder);

    configPivotMotors();

    m_armPid = new ProfiledPIDController(Constants.Arm.KPArmPivot, Constants.Arm.KIArmPivot, Constants.Arm.KDArmPivot, new TrapezoidProfile.Constraints(370, 500));
    m_armPid.reset(getPivotAngle(), 0);

    m_mechanism = new Mechanism2d(3, 3);
    m_mechanismRoot = m_mechanism.getRoot("A-Frame", 1, 1);
    m_armMech = m_mechanismRoot.append(new MechanismLigament2d("Arm", 1, getPivotAngle() - 106.4));

    if(Constants.DEBUG_MODE) {
      Constants.kDebugTab.add("Arm Mechanism", m_mechanism);
      Constants.kDebugTab.addBoolean("Arm IsAtSetpoint", () -> isArmAtSetpoint());
      Constants.kDebugTab.add("Arm PIDController", m_armPid);
    }

    m_pivotAngleSetpoint = getPivotAngle();
  }

  private void configPivotMotors() {    
    SparkMaxConfig leftconfig = new SparkMaxConfig();
    leftconfig
        .idleMode(IdleMode.kBrake)
        .closedLoopRampRate(0)
        .openLoopRampRate(0)
        .voltageCompensation(Constants.Arm.kPivotMotorVoltageLimit)
        .smartCurrentLimit(Constants.Arm.kPivotMotorCurrentLimit);

    SparkMaxConfig rightconfig = new SparkMaxConfig();
    rightconfig
        .idleMode(IdleMode.kBrake)
        .closedLoopRampRate(0)
        .openLoopRampRate(0)
        .voltageCompensation(Constants.Arm.kPivotMotorVoltageLimit)
        .follow(m_leftMotor, true);

    m_leftMotor.configure(leftconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightMotor.configure(rightconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public double getPivotAngle() {
    return m_armEncoder.get() * 360;
  }

  public boolean isArmAtSetpoint() {
    //if(DriverStation.isAutonomous()) return Math.abs(getPivotAngle() - m_pivotAngleSetpoint) <= 3.0;
    return Math.abs(getPivotAngle() - m_pivotAngleSetpoint) <= 5.0;
  }

  public void setArmSetpoint(double angle) {
    m_pivotAngleSetpoint = MathUtil.clamp(angle, Constants.Arm.kMinPivotAngle, Constants.Arm.kMaxPivotAngle);
  }

  private double calcPivotSpeed() {
    double setpoint = m_pivotAngleSetpoint;

    //if(setpoint < 110 && !isWristInSafeBounds()) setpoint = MathUtil.clamp(setpoint, 110, Constants.Arm.kMaxPivotAngle);

    double speed = m_armPid.calculate(getPivotAngle(), setpoint);

    if(getPivotAngle() > Constants.Arm.kMaxPivotAngle)
        speed = MathUtil.clamp(speed, -0.1, 0);
    else if(getPivotAngle() < Constants.Arm.kMinPivotAngle)
        speed = MathUtil.clamp(speed, 0, 0.1);

    return speed + m_feedforward.calculate(Units.degreesToRadians(getPivotAngle() - 106.4), 0);
  }

  public double getPivotSetpoint() {
    return m_pivotAngleSetpoint;
  }

  @Override
  public void periodic() {

    m_armMech.setAngle(getPivotAngle() - 106.4);

    m_leftMotor.set(calcPivotSpeed());

    SmartDashboard.putNumber("Pivot/Angle", getPivotAngle());
    SmartDashboard.putNumber("Pivot/Setpoint", m_pivotAngleSetpoint);
    SmartDashboard.putNumber("Pivot/Voltage", m_leftMotor.getBusVoltage());
    SmartDashboard.putNumber("Pivot/Speed", m_leftMotor.get());
  }
    
}
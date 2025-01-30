package team1403.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
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
import team1403.robot.Constants.Arm;

public class ArmWrist extends SubsystemBase {
  // lead motor
  private final SparkMax m_leftMotor;
  // following motor
  private final SparkMax m_rightMotor;
  private final SparkMax m_wristMotor;
  private final DutyCycleEncoder m_armEncoder;
  private final DutyCycleEncoder m_wristEncoder;
  private final ProfiledPIDController m_armPid;
  private final PIDController m_wristPid;
  private final ArmFeedforward m_feedforward;

  // Setpoints
  private double m_pivotAngleSetpoint;
  private double m_wristAngleSetpoint;


  private Mechanism2d m_mechanism;
  private MechanismRoot2d m_mechanismRoot;
  private MechanismLigament2d m_armMech;
  private MechanismLigament2d m_wristMech;

  public ArmWrist() {
    m_feedforward = new ArmFeedforward(0, Constants.Arm.kFeedforwardG, Constants.Arm.kFeedforwardV);
    m_leftMotor = new SparkMax(Constants.CanBus.leftPivotMotorID, MotorType.kBrushless);
    m_rightMotor = new SparkMax(Constants.CanBus.rightPivotMotorID, MotorType.kBrushless);
    m_wristMotor = new SparkMax(Constants.CanBus.wristMotorID, MotorType.kBrushless);
    m_armEncoder = new DutyCycleEncoder(Constants.RioPorts.kArmAbsoluteEncoder);
    m_wristEncoder = new DutyCycleEncoder(Constants.RioPorts.kwristAbsoluteEncoder);

    configMotors();

    m_armPid = new ProfiledPIDController(Constants.Arm.KPArmPivot, Constants.Arm.KIArmPivot, Constants.Arm.KDArmPivot, new TrapezoidProfile.Constraints(370, 500));
    m_wristPid = new PIDController(Constants.Wrist.KPWrist, Constants.Wrist.KIWrist, Constants.Wrist.KDWrist);
    m_armPid.reset(getPivotAngle(), 0);
    //m_wristPid.reset(getWristAngle(), 0);

    m_mechanism = new Mechanism2d(3, 3);
    m_mechanismRoot = m_mechanism.getRoot("A-Frame", 1, 1);
    m_armMech = m_mechanismRoot.append(new MechanismLigament2d("Arm", 1, getPivotAngle() - 106.4));
    m_wristMech = m_armMech.append(new MechanismLigament2d("Wrist", 0.3, getWristAngle()));

    if(Constants.DEBUG_MODE) {
      SmartDashboard.putData("Arm Mechanism", m_mechanism);
      SmartDashboard.putData("Arm PIDController", m_armPid);
      SmartDashboard.putData("Wrist PIDController", m_wristPid);
    }

    m_pivotAngleSetpoint = getPivotAngle();
    m_wristAngleSetpoint = Constants.Wrist.kIntakeSetpoint;
  }

  private void configMotors() {
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

    SparkMaxConfig wristconfig = new SparkMaxConfig();
    wristconfig
        .idleMode(IdleMode.kBrake);

    m_wristMotor.configure(wristconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public double getPivotAngle() {
    return m_armEncoder.get() * 360;
  }

  public double getWristAngle() {
    return m_wristEncoder.get() * 360;
  }

  public boolean isArmAtSetpoint() {
    return Math.abs(getPivotAngle() - m_pivotAngleSetpoint) <= 5.0;
  }

  public boolean isWristAtSetpoint() {
    return Math.abs(getWristAngle() - m_wristAngleSetpoint) <= 5.0;
  }

  public boolean isArmAndWristAtSetpoint() {
    return isArmAtSetpoint() && isWristAtSetpoint();
  }

  public void setArmSetpoint(double angle) {
    m_pivotAngleSetpoint = MathUtil.clamp(angle, Constants.Arm.kMinPivotAngle, Constants.Arm.kMaxPivotAngle);
  }

  public void setWristSetpoint(double angle) {
    m_wristAngleSetpoint = MathUtil.clamp(angle, Constants.Wrist.kWristLowerLimit, Constants.Wrist.kWristUpperLimit);
  }

  private boolean isWristInSafeBounds() {
    return getWristAngle() > 130 && getWristAngle() < 145;
  }

  private double calcPivotSpeed() {
    double setpoint = getPivotAngle();

    if(setpoint < 110 && !isWristInSafeBounds()) setpoint = MathUtil.clamp(setpoint, 110, Constants.Arm.kMaxPivotAngle);

    double speed = m_armPid.calculate(getPivotAngle(), setpoint);

    if(getPivotAngle() > Constants.Arm.kMaxPivotAngle)
        speed = MathUtil.clamp(speed, -0.1, 0);
    else if(getPivotAngle() < Constants.Arm.kMinPivotAngle)
        speed = MathUtil.clamp(speed, 0, 0.1);

    return speed + m_feedforward.calculate(Units.degreesToRadians(getPivotAngle() - 106.4), 0);
  }

  private double calcWristSpeed() {
    double setpoint = m_wristAngleSetpoint;

    if (getPivotAngle() < 110) setpoint = MathUtil.clamp(setpoint, Constants.Wrist.kIntakeSetpoint, 140);

    double speed = m_wristPid.calculate(getWristAngle(), setpoint);

    if(getWristAngle() < Constants.Wrist.kWristLowerLimit)
        speed = MathUtil.clamp(speed, 0, 0.1);
    else if(getWristAngle() > Constants.Wrist.kWristUpperLimit)
        speed = MathUtil.clamp(speed, -0.1, 0);

    return speed;
  }


  
  // Start of's wonderful code
  
  
  
  

  
public double ArmandWristpoint(double armAngle, double wristAngle) {
  setArmSetpoint(armAngle);
  setWristSetpoint(wristAngle);
  
  if (!isWristInSafeBounds()) {
    setWristSetpoint(145);
  }
  
  if (!isWristInSafeBounds()) {
    setArmSetpoint(armAngle);
  }
  
  return wristAngle;
}
  







  @Override
  public void periodic() {

    SmartDashboard.putBoolean("Wrist is at Setpoint", isWristAtSetpoint());
    SmartDashboard.putBoolean("Arm IsAtSetpoint", isArmAtSetpoint());

    m_leftMotor.set(calcPivotSpeed());
    m_wristMotor.set(calcWristSpeed());


    Logger.recordOutput("Pivot/Angle", getPivotAngle());
    Logger.recordOutput("Pivot/Setpoint", m_pivotAngleSetpoint);
    Logger.recordOutput("Pivot/Voltage", m_leftMotor.getBusVoltage());
    Logger.recordOutput("Pivot/Speed", m_leftMotor.get());
    Logger.recordOutput("Wrist/Voltage", m_wristMotor.getBusVoltage());
    Logger.recordOutput("Wrist/Current", m_wristMotor.getOutputCurrent());
    Logger.recordOutput("Wrist/Temp", m_wristMotor.getMotorTemperature());
    Logger.recordOutput("Wrist/Motor RPM", m_wristMotor.getEncoder().getVelocity());
    Logger.recordOutput("Wrist/Speed", m_wristMotor.get());
    Logger.recordOutput("Wrist/Angle", getWristAngle());
    Logger.recordOutput("Wrist/Setpoint", m_wristAngleSetpoint);
  }
    
}
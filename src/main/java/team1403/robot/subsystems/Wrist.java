package team1403.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
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

public class Wrist extends SubsystemBase {
  private final SparkMax m_wristMotor;
  private final DutyCycleEncoder m_wristEncoder;
  private final ProfiledPIDController m_wristPid;

  // Setpoints
  private double m_wristAngleSetpoint;

  public Wrist() {
    m_wristMotor = new SparkMax(Constants.CanBus.wristMotorID, MotorType.kBrushless);
    m_wristEncoder = new DutyCycleEncoder(Constants.RioPorts.kwristAbsoluteEncoder);

    configWristMotor();

    m_wristPid = new ProfiledPIDController(Constants.Wrist.KPWrist, Constants.Wrist.KIWrist, Constants.Wrist.KDWrist, new TrapezoidProfile.Constraints(250, 500));
    m_wristPid.reset(getWristAngle(), 0);
    m_wristPid.setIntegratorRange(-1, 1);

    if(Constants.DEBUG_MODE) {
      Constants.kDebugTab.addBoolean("Wrist is at Setpoint", () -> isWristAtSetpoint());
      Constants.kDebugTab.add("Wrist PIDController", m_wristPid);
    }

    m_wristAngleSetpoint = Constants.Wrist.kIntakeSetpoint;
  }

  private void configWristMotor() {
    SparkMaxConfig wristconfig = new SparkMaxConfig();
    wristconfig
        .idleMode(IdleMode.kBrake);

    m_wristMotor.configure(wristconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public double getWristAngle() {
    return m_wristEncoder.get() * 360;
  }

  public boolean isWristAtSetpoint() {
    //if(DriverStation.isAutonomous()) return Math.abs(getWristAngle() - m_wristAngleSetpoint) <= 3.0;
    return Math.abs(getWristAngle() - m_wristAngleSetpoint) <= 5.0;
  }

  public void setWristSetpoint(double angle) {
    m_wristAngleSetpoint = MathUtil.clamp(angle, Constants.Wrist.kWristLowerLimit, Constants.Wrist.kWristUpperLimit);
    //m_wristPid.reset(getWristAngle(), 0);
  }

  private boolean isWristInSafeBounds() {
    return getWristAngle() > 130 && getWristAngle() < 145;
  }

  private double calcWristSpeed() {
    double setpoint = m_wristAngleSetpoint;

    setpoint = MathUtil.clamp(setpoint, Constants.Wrist.kIntakeSetpoint, 140);

    double speed = m_wristPid.calculate(getWristAngle(), setpoint);

    if(getWristAngle() < Constants.Wrist.kWristLowerLimit)
        speed = MathUtil.clamp(speed, 0, 0.1);
    else if(getWristAngle() > Constants.Wrist.kWristUpperLimit)
        speed = MathUtil.clamp(speed, -0.1, 0);

    return speed;
  }

  @Override
  public void periodic() {

    m_wristMotor.set(calcWristSpeed());

    SmartDashboard.putNumber("Wrist/Voltage", m_wristMotor.getBusVoltage());
    SmartDashboard.putNumber("Wrist/Current", m_wristMotor.getOutputCurrent());
    SmartDashboard.putNumber("Wrist/Temp", m_wristMotor.getMotorTemperature());
    SmartDashboard.putNumber("Wrist/Motor RPM", m_wristMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Wrist/Speed", m_wristMotor.get());
    SmartDashboard.putNumber("Wrist/Angle", getWristAngle());
    SmartDashboard.putNumber("Wrist/Setpoint", m_wristAngleSetpoint);
  }
    
}
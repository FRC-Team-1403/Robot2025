package team1403.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;
import dev.doglog.DogLog;
import edu.wpi.first.util.datalog.DataLog;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.robot.Constants;

public class Elevator extends SubsystemBase {
  private SparkMax m_leftMotor; 
  private SparkMax m_rightMotor;
  private ProfiledPIDController control; 

  public Elevator() {
    m_leftMotor = new SparkMax(Constants.Elevator.leftMotor, MotorType.kBrushless);
    m_rightMotor = new SparkMax(Constants.Elevator.rightMotor, MotorType.kBrushless);
    // m_leftMotor.restoreFactoryDefaults();
    // m_rightMotor.restoreFactoryDefaults();
    // m_rightMotor.setIdleMode(IdleMode.kBrake);
    // m_leftMotor.setIdleMode(IdleMode.kBrake);
    // m_rightMotor.setInverted(true);
    // m_leftMotor.setInverted(false);

    m_leftMotor.getEncoder().setPosition(0);
    m_rightMotor.getEncoder().setPosition(0);
    control = new ProfiledPIDController(
      Constants.Elevator.kPSparkMax,
      Constants.Elevator.kISparkMax,
      Constants.Elevator.kDSparkMax, 
      new TrapezoidProfile.Constraints(5, 10)
    );

  }
  
  public void setMotorSpeed(double speed) {
    m_leftMotor.set(MathUtil.clamp(speed, -1, 1));
    m_rightMotor.set(MathUtil.clamp(speed, -1, 1));
  }

  public void stopMotors() {
    setMotorSpeed(0);
  }
  
  public double getSpeed() {
    return m_rightMotor.get();
  }

  public void moveToSetPoint(double goal) {
    m_leftMotor.set(control.calculate(m_leftMotor.getEncoder().getPosition(), goal));
    m_rightMotor.set(control.calculate(m_rightMotor.getEncoder().getPosition(), goal));
  }
  
   

  public void periodic() {
    DogLog.log("Left Motor Encoder", m_leftMotor.getEncoder().getPosition());
    DogLog.log("Right Motor Encoder", m_rightMotor.getEncoder().getPosition());
    Logger.recordOutput("Left Motor Speed", m_leftMotor.get());
    Logger.recordOutput("Right Motor Speed", m_rightMotor.get());
  }
}
package team1403.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;
import dev.doglog.DogLog;
import edu.wpi.first.util.datalog.DataLog;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.robot.Constants;

public class Elevator extends SubsystemBase {
  private SparkMax m_leftMotor;
  private SparkMax m_rightMotor;
  private final ElevatorFeedforward m_ElevatorFeedforward;

  public Elevator() {
    // m_leftMotor = new SparkMax(Constants.CanBus.leftHangerMotorID, MotorType.kBrushless);
    // m_rightMotor = new SparkMax(Constants.CanBus.rightHangerMotorID, MotorType.kBrushless);

    // // m_leftMotor.restoreFactoryDefaults();
    // // m_rightMotor.restoreFactoryDefaults();
    // // m_rightMotor.setIdleMode(IdleMode.kBrake);
    // // m_leftMotor.setIdleMode(IdleMode.kBrake);
    // // m_rightMotor.setInverted(true);
    // // m_leftMotor.setInverted(false);

    // m_leftMotor.getEncoder().setPosition(0);
    // m_rightMotor.getEncoder().setPosition(0);
    m_ElevatorFeedforward = new ElevatorFeedforward(0, Constants.Elevator.kFeedforwardG, Constants.Elevator.kFeedforwardV, 0, Constants.kLoopTime);
  }
  
  public void setMotorSpeed(double speed) {
    // m_leftMotor.set(MathUtil.clamp(speed, -1, 1));
    // m_rightMotor.set(MathUtil.clamp(speed, -1, 1));
    // m_leftMotor.set(speed);
    // m_rightMotor.set(speed);
  }

  public void stopMotors() {
    setMotorSpeed(0);
  }
  
  public double getSpeed() {
    return m_leftMotor.get();
  }

  public double getPosition() {
    return m_rightMotor.getEncoder().getPosition();
  }

  public double calculation(double pos, double setpoint) {
    return m_ElevatorFeedforward.calculate(pos, setpoint);
  }

  public void periodic() {

    // Logger.recordOutput("Left Motor Encoder", m_leftMotor.getEncoder().getPosition());
    // Logger.recordOutput("Right Motor Encoder", m_rightMotor.getEncoder().getPosition());
    // Logger.recordOutput("Left Motor Speed", m_leftMotor.get());
    // Logger.recordOutput("Right Motor Speed", m_rightMotor.get());
  }
}
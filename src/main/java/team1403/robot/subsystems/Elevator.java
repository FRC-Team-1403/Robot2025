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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.robot.Constants;

public class Elevator extends SubsystemBase { 
  public static SparkMax m_motor;
  private ProfiledPIDController control; 

  public static final ElevatorSim m_elevatorSim = new ElevatorSim(DCMotor.getNEO(1),1/9, 6.80389, 1.751, 0, 0.762, true, 0, 0.01, 0.0);

  public Elevator() {
    m_motor = new SparkMax(Constants.Elevator.rightMotor, MotorType.kBrushless);
    // m_leftMotor.restoreFactoryDefaults();
    // m_rightMotor.restoreFactoryDefaults();
    // m_rightMotor.setIdleMode(IdleMode.kBrake);
    // m_leftMotor.setIdleMode(IdleMode.kBrake);
    // m_rightMotor.setInverted(true);
    // m_leftMotor.setInverted(false);

    m_motor.getEncoder().setPosition(0);
    control = new ProfiledPIDController(
      Constants.Elevator.kPSparkMax,
      Constants.Elevator.kISparkMax,
      Constants.Elevator.kDSparkMax, 
      new TrapezoidProfile.Constraints(5, 10)
    );

  }
  
  public void setMotorSpeed(double speed) {
    m_motor.set(MathUtil.clamp(speed, -1, 1));
  }

  public void stopMotors() {
    setMotorSpeed(0);
  }
  
  public double getSpeed() {
    return m_motor.get();
  }

  public void moveToSetPoint(double goal) {
    m_motor.set(control.calculate(m_motor.getEncoder().getPosition(), goal));
  }
  
   

  public void periodic() {
    DogLog.log("Motor Encoder", m_motor.getEncoder().getPosition());
    DogLog.log("Right Motor Speed", m_motor.get());
  }

  @Override
  public void simulationPeriodic() {

    Elevator.m_elevatorSim.setInputVoltage(Elevator.m_motor.get() * RobotController.getBatteryVoltage());
    Elevator.m_elevatorSim.update(0.020);
    Elevator.m_motor.getEncoder().setPosition(Elevator.m_elevatorSim.getPositionMeters());
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(Elevator.m_elevatorSim.getCurrentDrawAmps()));
    
  }

}
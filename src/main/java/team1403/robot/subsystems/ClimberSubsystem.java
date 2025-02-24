package team1403.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import team1403.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  private final SparkMax m_leftMotor;
  private final SparkMax m_rightMotor;
  private final ProfiledPIDController m_pid;

  public ClimberSubsystem() {
    m_leftMotor = new SparkMax(Constants.CanBus.climberLeftMotor, MotorType.kBrushless);
    m_rightMotor = new SparkMax(Constants.CanBus.climberRightMotor, MotorType.kBrushless);
    configMotors();
    m_pid = new ProfiledPIDController(0.1, 0, 0, new TrapezoidProfile.Constraints(Constants.Climber.maxVelo, Constants.Climber.maxAccel));
  }

  private void configMotors() {
    SparkMaxConfig leftconfig = new SparkMaxConfig();
    leftconfig
        .idleMode(IdleMode.kBrake)
        .follow(m_rightMotor);
    m_leftMotor.configure(leftconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    SparkMaxConfig rightconfig = new SparkMaxConfig();
    rightconfig
        .idleMode(IdleMode.kBrake)
        .inverted(true);
    m_rightMotor.configure(rightconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public double getAngle() {
    return m_rightMotor.getEncoder().getPosition() * 360;
  }

  public void setClimberAngle(double targetAngle) {
        m_pid.setGoal(targetAngle);
    }

  public boolean isAtSetpoint() {
      return Math.abs(getAngle() - m_pid.getGoal().position) 
          < Units.degreesToRotations(5);
  }

  @Override
  public void periodic() {
    m_rightMotor.set(m_pid.calculate(getAngle()));

    Logger.recordOutput("Climber at setpoint", isAtSetpoint());
    Logger.recordOutput("Climber Angle", getAngle());
  }
}
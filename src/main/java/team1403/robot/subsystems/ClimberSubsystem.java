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

  public ClimberSubsystem() {
    m_leftMotor = new SparkMax(Constants.CanBus.climberLeftMotor, MotorType.kBrushless);
    m_rightMotor = new SparkMax(Constants.CanBus.climberRightMotor, MotorType.kBrushless);
    configMotors();
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

  @Override
  public void periodic() {
  }
}
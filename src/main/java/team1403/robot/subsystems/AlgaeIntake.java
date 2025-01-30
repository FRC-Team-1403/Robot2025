package team1403.robot.subsystems;

import java.util.logging.Logger;

import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import dev.doglog.DogLog;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import team1403.robot.Constants;

public class AlgaeIntake extends SubsystemBase{
    private final SparkMax m_algaeIntakeMotor;
    private DigitalInput m_algaeIntakePhotogate;
    private final MotionMagicVelocityDutyCycle m_request = new MotionMagicVelocityDutyCycle(0);
    private SysIdRoutine m_sysIdRoutine;
    
    public AlgaeIntake() {
        m_algaeIntakeMotor = new SparkMax(Constants.CanBus.intakeMotorID, MotorType.kBrushless);
        configMotors();
        m_algaeIntakePhotogate = new DigitalInput(Constants.RioPorts.intakePhotogate1);
    }
    private void configMotors() {
    SparkMaxConfig intakeconfig = new SparkMaxConfig();
    intakeconfig
        .idleMode(IdleMode.kBrake);
    m_algaeIntakeMotor.configure(intakeconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  public boolean isAlgaeIntakePhotogateTriggered() {
    return !m_algaeIntakePhotogate.get();
  }
  
  public void intakeStop() {
    m_algaeIntakeMotor.set(0);
  }
   

 public void setIntakeSpeed(double speed) {
   
   m_algaeIntakeMotor.set(speed);
}
public boolean hasAlgae() {
    return isAlgaeIntakePhotogateTriggered();
  }

  public double algaeIntakeSpeed() {
    return m_algaeIntakeMotor.get();
  }

  // Logger.recordOutput("Intake/Motor Temp", m_algaeIntakeMotor.getMotorTemperature());
  // Logger.recordOutput("Intake/gate", isAlgaeIntakePhotogateTriggered());
  // Logger.recordOutput("Is Loaded", isLoaded());
  // Logger.recordOutput("Intake/Speed Setpoint", m_algaeintakeMotor.get());
  // Logger.recordOutput("Intake/Current", m_algaeintakeMotor.getOutputCurrent());
  
}
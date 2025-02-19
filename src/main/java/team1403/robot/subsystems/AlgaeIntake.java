package team1403.robot.subsystems;

import java.util.logging.Logger;

import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import team1403.robot.Constants;

public class AlgaeIntake extends SubsystemBase{
    private SparkMax m_algaeIntakeMotor;
    private SparkMax m_elbowMotor;
    private DigitalInput m_photogate;
    private SysIdRoutine m_sysIdRoutine;
    
    public AlgaeIntake() {
        m_algaeIntakeMotor = new SparkMax(Constants.CanBus.algaeIntakeMotorID, MotorType.kBrushless);
        m_elbowMotor = new SparkMax(Constants.CanBus.elbowMotorID, MotorType.kBrushless);
        configMotors();
        m_photogate = new DigitalInput(Constants.RioPorts.algaeIntakePhotogateID);
    }
    private void configMotors() {
    SparkMaxConfig intakeconfig = new SparkMaxConfig();
    intakeconfig
        .idleMode(IdleMode.kBrake);
    m_algaeIntakeMotor.configure(intakeconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  public boolean isAlgaeIntaked() {
    return !m_photogate.get();
  }
  
  public void intakeStop() {
    m_algaeIntakeMotor.set(0);
  }

  public void setIntakeSpeed(double speed) {
    m_algaeIntakeMotor.set(speed);
  }

  public double getAlgaeIntakeSpeed() {
    return m_algaeIntakeMotor.get();
  }

  public void setElbowSpeed(double speed) {
    m_elbowMotor.set(speed);
  }

  public void stopElbow() {
    m_elbowMotor.set(0);
  }

  public boolean isReady() {
    return (getAngle() == Constants.AlgaeIntake.upPos && !isAlgaeIntaked() && m_elbowMotor.get() == 0);
  }

  public double getAngle() {
    return m_elbowMotor.getEncoder().getPosition();
  }

}
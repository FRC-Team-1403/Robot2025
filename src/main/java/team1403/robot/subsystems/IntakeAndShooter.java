package team1403.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import team1403.robot.Constants;
// import team1403.robot.swerve.ISwerveModule;
// import team1403.robot.swerve.ISwerveModule.DriveControlType;
// import team1403.robot.swerve.ISwerveModule.SteerControlType;

import static edu.wpi.first.units.Units.Volts;

/**
 * creating the intake and shooter class.
 */
public class IntakeAndShooter extends SubsystemBase {  
  // Intake motor
  private static SparkMax m_intakeMotor;
  
  // shooter motors
  private TalonFX m_shooterMotorTop;
  private TalonFX m_shooterMotorBottom;
  private Double m_topVel;
  private Double m_bottomVel;

  // photogates
  private DigitalInput m_intakePhotogate;
  private DigitalInput m_shooterPhotogate;

  private final MotionMagicVelocityDutyCycle m_request = new MotionMagicVelocityDutyCycle(0);

  private SysIdRoutine m_sysIdRoutine;


  /**
   * creating shooter devices.
   *
   * @param injectedParameters
   */
  /**
   * creating motor and sensors for the intake.
   *
   * @param injectedParameters injected parameters.
   */
  public IntakeAndShooter() {
    // intake motors and sensors
    m_intakeMotor = new SparkMax(Constants.CanBus.intakeMotorID, MotorType.kBrushless);
    configMotors();
    m_intakePhotogate = new DigitalInput(Constants.RioPorts.intakePhotogate1);
    // shooter motors and sensors
    m_shooterMotorTop = new TalonFX(Constants.CanBus.shooterMotorTopID);
    m_shooterMotorBottom = new TalonFX(Constants.CanBus.shooterMotorBottomID);
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotionMagic.MotionMagicAcceleration = 2000; // RPM/s -> ~0.5 s to max

    // FIXME: Tune these values!
    var slot0Configs = config.Slot0;
    slot0Configs.kS = 0;
    slot0Configs.kV = 0.0092;
    slot0Configs.kA = 0;
    slot0Configs.kP = 0.003;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;

    m_shooterMotorTop.getConfigurator().apply(config);
    m_shooterMotorBottom.getConfigurator().apply(config);

    m_shooterPhotogate = new DigitalInput(Constants.RioPorts.shooterPhotogate);

    m_topVel = m_shooterMotorTop.getVelocity().getValueAsDouble();
    m_bottomVel = m_shooterMotorBottom.getVelocity().getValueAsDouble();

    if(Constants.DEBUG_MODE) {
      Constants.kDebugTab.addBoolean("Intake Sensor", () -> isIntakePhotogateTriggered());
      Constants.kDebugTab.addBoolean("Shooter Sensor", () -> isShooterPhotogateTriggered());
      Constants.kDebugTab.addBoolean("Shooter (teleop) Ready", () -> teleopIsReady());
    }

    m_sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(null, null, null, 
      (state) -> Logger.recordOutput("SysIDSwerveLinear", state.toString())),
      new SysIdRoutine.Mechanism((voltage) -> {
        m_shooterMotorBottom.setVoltage(voltage.in(Volts));
        m_shooterMotorTop.setVoltage(voltage.in(Volts));
    }, null, this));
  }
  
  private void updateVelocity() {
    m_topVel = m_shooterMotorTop.getVelocity().getValueAsDouble();
    m_bottomVel = m_shooterMotorBottom.getVelocity().getValueAsDouble();
  }

  private void configMotors() {
    SparkMaxConfig intakeconfig = new SparkMaxConfig();
    intakeconfig
        .idleMode(IdleMode.kBrake);
    m_intakeMotor.configure(intakeconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }


  /**
   * state of the intake.
   *
   * @return true or false depending on if it is trigered.
   */
  public boolean isIntakePhotogateTriggered() {
    return !m_intakePhotogate.get();
  }

  /**
   * stopping intake.
   */
  public void intakeStop() {
    m_intakeMotor.set(0);
  }

  /**
   * setting intake speed
   *
   * @param speed
   */
  public void setIntakeSpeed(double speed) {
    // if (m_intakeLimitSwitch.get()) {
    // stop();
    // return;
    // }
    m_intakeMotor.set(speed);
    // if there is an error when testing (note doesn't get taken in) try changing
    // the direction of the motor

  }

  /**
   * is shooter photogate triggered.
   *
   * @return state of the shooter photogate.
   */
  public boolean isShooterPhotogateTriggered() {
    return !m_shooterPhotogate.get();
  }

  public void setShooterRPM(double rpm) {
    m_request.Velocity = rpm / 60.0;
  }

  /**
   * Stopping the shooter motors.
   */
  public void shooterStop() {
    m_request.Velocity = 0;
  }

  public boolean isReady(){
    return Math.abs(m_request.Velocity * 60 - m_bottomVel * 60) < 300 && 
           Math.abs(m_request.Velocity * 60 - m_topVel * 60) < 300;
  }

  public boolean teleopIsReady() {
    return Math.abs(m_request.Velocity * 60 - m_bottomVel * 60) < 1000;
  }

  public boolean isLoaded() {
    return isIntakePhotogateTriggered() && !isShooterPhotogateTriggered();
  }

  public double intakeSpeed() {
    return m_intakeMotor.get();
  }

  public Command getSysIDQ(SysIdRoutine.Direction dir) {
    return m_sysIdRoutine.quasistatic(dir);
  }

  public Command getSysIDD(SysIdRoutine.Direction dir) {
    return m_sysIdRoutine.dynamic(dir);
  }

  public void periodic() {
    updateVelocity();
    m_shooterMotorTop.setControl(m_request);
    m_shooterMotorBottom.setControl(m_request);

    //Blackbox.setLoaded(isIntakePhotogateTriggered() && !isShooterPhotogateTriggered());

    Logger.recordOutput("Intake/Motor Temp", m_intakeMotor.getMotorTemperature());
    Logger.recordOutput("Shooter/Speed", m_shooterMotorTop.get());
    Logger.recordOutput("Shooter/Voltage", m_shooterMotorTop.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput("Shooter/gate", isShooterPhotogateTriggered());
    Logger.recordOutput("Intake/gate", isIntakePhotogateTriggered());
    Logger.recordOutput("Shooter/top Motor RPM", m_topVel);
    Logger.recordOutput("Shooter/bottom Motor RPM", m_bottomVel);
    Logger.recordOutput("Is Loaded", isLoaded());
    // log("Intake/RPM", m_intakeMotor.get());
    Logger.recordOutput("Intake/Speed Setpoint", m_intakeMotor.get());
    Logger.recordOutput("Shooter/RPM setpoint",  m_request.Velocity);
    Logger.recordOutput("Intake/Current", m_intakeMotor.getOutputCurrent());
    Logger.recordOutput("Shooter/top current", m_shooterMotorTop.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Shooter/bottom current", m_shooterMotorBottom.getStatorCurrent().getValueAsDouble());
  }
}
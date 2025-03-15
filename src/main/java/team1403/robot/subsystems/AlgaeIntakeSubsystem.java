// package team1403.robot.subsystems;

// import org.littletonrobotics.junction.Logger;

// import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
// import com.pathplanner.lib.config.PIDConstants;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.config.SparkMaxConfig;

// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.motorcontrol.Spark;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
// import team1403.robot.Constants;

// public class AlgaeIntakeSubsystem extends SubsystemBase {
//   private final SparkMax m_algaeIntakeMotor;
//   private final SparkMax m_algaeWristMotor;
//   private final DigitalInput m_algaeIntakePhotogate;
//   private final ProfiledPIDController m_pid;

//   public AlgaeIntakeSubsystem() {
//     m_algaeIntakeMotor = new SparkMax(Constants.CanBus.algaeIntakeMotorID, MotorType.kBrushless);
//     m_algaeWristMotor = new SparkMax(Constants.CanBus.algaeWristMotorID, MotorType.kBrushless);
//     m_algaeIntakePhotogate = new DigitalInput(Constants.RioPorts.kAlgaeIntakePhotogateID);
//     configMotors();
//     m_pid = new ProfiledPIDController(0.1, 0, 0, new TrapezoidProfile.Constraints(Constants.AlgaeIntake.maxVelo, Constants.AlgaeIntake.maxAccel));
//   }

//   private void configMotors() {
//     SparkMaxConfig intakeconfig = new SparkMaxConfig();
//     intakeconfig
//         .idleMode(IdleMode.kBrake);
//     m_algaeIntakeMotor.configure(intakeconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
//     SparkMaxConfig wristconfig = new SparkMaxConfig();
//     wristconfig
//         .idleMode(IdleMode.kBrake);
//     m_algaeIntakeMotor.configure(wristconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//   }

//   public void intakeStop() {
//     m_algaeIntakeMotor.set(0);
//   }

//   public void setIntakeSpeed(double speed) {
//     m_algaeIntakeMotor.set(speed);
//   }

//   public double getIntakeSpeed() {
//     return m_algaeIntakeMotor.get();
//   }

//   public boolean hasAlgae() {
//     return !m_algaeIntakePhotogate.get();
//   }

//   public void setWristSpeed(double speed) {
//     m_algaeWristMotor.set(speed);
//   }

//   public double getWristSpeed() {
//     return m_algaeWristMotor.get();
//   }

//   public double getWristAngle() {
//     return m_algaeWristMotor.getEncoder().getPosition() * 360.0;
//   }

//   public void setWristAngle(double targetAngle) {
//         m_pid.setGoal(targetAngle);
//     }

//   public boolean isAtSetpoint() {
//       return Math.abs(getWristAngle() - m_pid.getGoal().position) 
//           < Units.degreesToRotations(5);
//   }

//   @Override
//   public void periodic() {
//     m_algaeWristMotor.set(m_pid.calculate(getWristAngle()));

//     Logger.recordOutput("Algae Intake Motor Temp", m_algaeIntakeMotor.getMotorTemperature());
//     Logger.recordOutput("Has algae", hasAlgae());
//     Logger.recordOutput("Algae Intake Speed", getIntakeSpeed());
//   }
// }
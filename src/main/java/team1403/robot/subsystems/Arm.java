// package team1403.robot.subsystems;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import org.littletonrobotics.junction.Logger;
// import dev.doglog.DogLog;

// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkMax;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;

// import team1403.robot.Constants;

// public class Arm extends SubsystemBase {
//   private SparkMax m_motor; 
//   private ProfiledPIDController PID; 

//   public Arm() {
//     m_motor = new SparkMax(Constants.Arm.kmotor, MotorType.kBrushless);
//     // m_leftMotor.restoreFactoryDefaults();
//     // m_rightMotor.restoreFactoryDefaults();
//     // m_rightMotor.setIdleMode(IdleMode.kBrake);
//     // m_leftMotor.setIdleMode(IdleMode.kBrake);
//     // m_rightMotor.setInverted(true);
//     // m_leftMotor.setInverted(false);

//     m_motor.getEncoder().setPosition(0);
//     PID = new ProfiledPIDController(
//       Constants.Elevator.kPSparkMax,
//       Constants.Elevator.kISparkMax,
//       Constants.Elevator.kDSparkMax, 
//       new TrapezoidProfile.Constraints(5, 10)
//     );

//   }
  
//   public void setMotorSpeed(double speed) {
//     m_motor.set(MathUtil.clamp(speed, -1, 1));
//   }

//   public void stopMotors() {
//     setMotorSpeed(0);
//   }
  
//   public double getSpeed() {
//     return m_motor.get();
//   }

//   public void moveToSetPoint(double goal) {
//     m_motor.set(PID.calculate(m_motor.getEncoder().getPosition(), goal));
//   }
  
   

//   public void periodic() {
//     DogLog.log("Left Motor Encoder", m_motor.getEncoder().getPosition());
//     DogLog.log("Left Motor Speed", m_motor.get());

//   }
// }

package team1403.robot.subsystems;


import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;


import org.littletonrobotics.junction.AutoLogOutput;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.robot.Constants;


public class WristSubsystem extends SubsystemBase {


  private final SparkMax m_wristMotor;
  private final ProfiledPIDController m_wristPID;
  private double angleSetpoint;


   public AlgaeWristSubsystem() {
       m_wristMotor = new SparkMax(Constants.CanBus.algaeWristMotorID, MotorType.kBrushless);
       m_wristPID = new ProfiledPIDController(Constants.AlgaeWrist.Kp, Constants.AlgaeWrist.Ki, Constants.AlgaeWrist.Kd, new TrapezoidProfile.Constraints(Constants.AlgaeIntake.maxVelo, Constants.AlgaeIntake.maxAccel));


       SparkMaxConfig wConfig = new SparkMaxConfig();
       wConfig.smartCurrentLimit(40);
       m_wristMotor.configure(wConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
   }


   public void setWristAngle(double angleSetpoint) {
       this.angleSetpoint = angleSetpoint;
       m_wristPID.setGoal(angleSetpoint);
   }  


   @Override
   public void periodic() {
      
   }


}

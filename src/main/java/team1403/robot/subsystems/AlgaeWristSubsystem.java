package team1403.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.robot.Constants;


public class AlgaeWristSubsystem extends SubsystemBase {

   private final SparkMax m_wristMotor;
   private final ProfiledPIDController m_wristPID;
   private double angleSetpoint;
   private final SimpleMotorFeedforward m_Feedforward;
   private double positionValue; 

    public AlgaeWristSubsystem() {
        m_wristMotor = new SparkMax(Constants.CanBus.algaeWristMotorID, MotorType.kBrushless);
        m_wristPID = new ProfiledPIDController(Constants.AlgaeWrist.Kp, Constants.AlgaeWrist.Ki, Constants.AlgaeWrist.Kd, new TrapezoidProfile.Constraints(Constants.AlgaeIntake.maxVelo, Constants.AlgaeIntake.maxAccel));
        m_Feedforward = new SimpleMotorFeedforward(Constants.Wrist.WristKS, Constants.Wrist.WristKS, 0); //change later
        positionValue = m_wristMotor.getAbsoluteEncoder().getPosition();
        SparkMaxConfig wConfig = new SparkMaxConfig();
        wConfig.smartCurrentLimit(40);
        m_wristMotor.configure(wConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    }

    public void setWristAngle(double angleSetpoint) {
        this.angleSetpoint = angleSetpoint;
        m_wristPID.setGoal(angleSetpoint);
    }   

    public void defaultWristCommand() { if (positionValue - angleSetpoint > 0.1) { setWristAngle(angleSetpoint); } }

   

    @Override
    public void periodic() {
        defaultWristCommand();
        m_wristPID.calculate(angleSetpoint, m_Feedforward.getKa());
    }

}

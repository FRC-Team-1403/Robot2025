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
        m_Feedforward = new SimpleMotorFeedforward(Constants.Wrist.WristKS, Constants.Wrist.WristKV, 0); //change later
        SparkMaxConfig wConfig = new SparkMaxConfig();
        wConfig.smartCurrentLimit(40);
        m_wristMotor.configure(wConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    }

    public void setWristAngle(double angleSetpoint) {
        this.angleSetpoint = angleSetpoint;
        m_wristPID.setGoal(angleSetpoint);
    }   


    public double getWristAngle(){
        return m_wristMotor.getAbsoluteEncoder().getPosition() * (2 * Math.PI);
    }

    public boolean isAtSetpoint() {
        return m_wristPID.atGoal();
    }

    public double getVelocity(){
        return m_wristPID.getSetpoint().velocity;
    }

    @Override
    public void periodic() {
        double pidCalc = m_wristPID.calculate(getWristAngle());
        double feedForwardCalc = m_Feedforward.calculate(getVelocity());

        m_wristMotor.setVoltage(feedForwardCalc + pidCalc);


       
    }

    

}

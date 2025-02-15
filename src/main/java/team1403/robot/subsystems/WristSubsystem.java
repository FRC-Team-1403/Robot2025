package team1403.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.robot.Constants;

public class WristSubsystem extends SubsystemBase {
    private SparkMax m_wristMotor;
    private DutyCycleEncoder m_encoder;
    private ArmFeedforward m_feedForward;
    private PIDController m_wristPID;

    public WristSubsystem() {
        m_wristMotor = new SparkMax(Constants.CanBus.wristMotorID, MotorType.kBrushless);
        m_encoder = new DutyCycleEncoder(Constants.RioPorts.WristEncoderID);
        m_feedForward = new ArmFeedforward(Constants.Wrist.WristKS, Constants.Wrist.WristKG, Constants.Wrist.WristKV);
        m_wristPID = new PIDController(Constants.Wrist.WristKP, Constants.Wrist.WristKI, Constants.Wrist.WristKD);
    }

    public double getWristAngle(){
        return m_encoder.get();
    }

    public void setWristAngle(double targetAngle) {
        m_wristPID.calculate(getWristAngle(), targetAngle);
    }
}

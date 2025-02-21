package team1403.robot.subsystems;

import javax.imageio.plugins.tiff.TIFFDirectory;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.robot.Constants;

public class WristSubsystem extends SubsystemBase {
    private SparkMax m_wristMotor;
    private DutyCycleEncoder m_encoder;
    private ArmFeedforward m_feedForward;
    private PIDController m_wristPID;
    private TrapezoidProfile m_trapezoidPID;

    public WristSubsystem() {
        m_wristMotor = new SparkMax(Constants.CanBus.wristMotorID, MotorType.kBrushless);
        m_feedForward = new ArmFeedforward(Constants.Wrist.WristKS, Constants.Wrist.WristKG, Constants.Wrist.WristKV);
        m_wristPID = new PIDController(Constants.Wrist.WristKP, Constants.Wrist.WristKI, Constants.Wrist.WristKD);
        m_trapezoidPID = new TrapezoidProfile(new TrapezoidProfile.Constraints(Constants.Wrist.trapezoidVel*5,Constants.Wrist.trapezoidAcc*5));

        
    }
}

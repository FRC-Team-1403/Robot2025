package team1403.robot.subsystems;

//import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Ultrasonic;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import team1403.robot.Constants;


public class EndEffectorSubsystem extends SubsystemBase {
    private SparkMax m_intakeMotor; 
    private SparkMax m_wristMotor;
    private ArmFeedforward m_feedforward;
    private Ultrasonic m_ultrasonic;
    private DutyCycleEncoder m_encoder;

    //private final MotionMagicVelocityDutyCycle m_request = new MotionMagicVelocityDutyCycle(0);
    
    //private SysIdRoutine m_SysIdRoutine;
    
    public EndEffectorSubsystem() {
        m_intakeMotor = new SparkMax(Constants.Coral.intakeMotor, MotorType.kBrushless);
        m_wristMotor = new SparkMax(Constants.Coral.wristMotor, MotorType.kBrushless);
        m_ultrasonic = new Ultrasonic(Constants.Coral.pingChannel, Constants.Coral.echoChannel);
        m_encoder = new DutyCycleEncoder(0);
        m_feedforward = new ArmFeedforward(0, Constants.Coral.kFeedforwardG, Constants.Coral.kFeedforwardV);
    }

    public void setIntakeMotorSpeed(double speed) {
        m_intakeMotor.set(speed);
    }

    public void setWristMotorSpeed(double speed) {
        m_wristMotor.set(speed);
    }

    public double getDistance() {
        return m_ultrasonic.getRangeMM() / 1000.0; 
    }

    public boolean isIntakeLoaded() {

        if (m_ultrasonic.getRangeInches() <= 14.875) { // accounts for 1 inch error
            return true;
        } else {
            return false;
        }
    }

    public double getEncoderValue() {
        return m_encoder.get();
    }
}

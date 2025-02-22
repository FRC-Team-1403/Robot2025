package team1403.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.util.datalog.DataLog;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.robot.Constants;

public class ClimberSubsystem extends SubsystemBase{
    private SparkMax m_leftMotor;
    private SparkMax m_rightMotor;
    private double m_target;

    public ClimberSubsystem(){
        m_leftMotor = new SparkMax(Constants.Climber.leftMotor, MotorType.kBrushless);
        m_rightMotor = new SparkMax(Constants.Climber.rightMotor, MotorType.kBrushless);

        configMotors();
    }

    public void configMotors(){
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        leftConfig 
            .idleMode(IdleMode.kBrake)
            .follow(m_rightMotor);
        SparkMaxConfig righConfig = new SparkMaxConfig();
        righConfig 
            .idleMode(IdleMode.kBrake)
            .inverted(true);
    }

    public void stopMotors() {
        m_rightMotor.set(0);
    }

    public boolean atPos() {
        return Math.abs(m_target - m_rightMotor.getEncoder().getPosition() / 360.0) < 3.0;
    }

    public void lift(){
        m_target = Constants.Climber.liftPos / 360.0;
        if (!atPos()) {
            m_rightMotor.set(Constants.Climber.upSpeed);
        } 
        else {
            stopMotors();
        }
    }

    public void lower(){
        m_target = Constants.Climber.lowerPos / 360.0;
        if (!atPos()) {
            m_rightMotor.set(Constants.Climber.downSpeed);
        } 
        else {
            stopMotors();
        }
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Right Climber Angle", m_rightMotor.getEncoder().getPosition() / 360.0);
        Logger.recordOutput("Right Climber Speed", m_rightMotor.get());
    }

}

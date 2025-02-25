package team1403.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.util.datalog.DataLog;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.robot.Constants;

public class ClimberSubsystem extends SubsystemBase{
    private SparkMax m_leftMotor;
    private SparkMax m_rightMotor;

    public ClimberSubsystem(){
        m_leftMotor = new SparkMax(Constants.CanBus.leftClimberMotor, MotorType.kBrushless);
        m_rightMotor = new SparkMax(Constants.CanBus.rightClimberMotor, MotorType.kBrushless);
        configMotors();
    }

    public void configMotors(){
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        leftConfig 
            .idleMode(IdleMode.kBrake)
            .follow(m_rightMotor, true);
        SparkMaxConfig rightConfig = new SparkMaxConfig();
        rightConfig 
            .idleMode(IdleMode.kBrake);

        m_leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setMotorSpeed(double speed) {
        m_rightMotor.set(speed);
    }

    public void stopMotors() {
        m_rightMotor.set(0);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Right Climber Speed", m_rightMotor.get());
    }

}
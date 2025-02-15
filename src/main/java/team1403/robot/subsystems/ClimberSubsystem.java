package team1403.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.util.datalog.DataLog;

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
    private double speed;

    public ClimberSubsystem(){
        m_leftMotor = new SparkMax(Constants.Climber.leftMotor, MotorType.kBrushless);
        m_rightMotor = new SparkMax(Constants.Climber.rightMotor, MotorType.kBrushless);

        
    }

    public void configMotors(){
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        leftConfig 
            .idleMode(IdleMode.kBrake)
            .follow(m_rightMotor, true);

        SparkMaxConfig righConfig = new SparkMaxConfig();
        righConfig 
            .idleMode(IdleMode.kBrake)
            .inverted(true);
    }

    public void lift(double speed){
        m_leftMotor.set(speed);
        m_rightMotor.set(speed);
    }

    public void lower(double speed){
        m_leftMotor.set(-speed); 
        m_rightMotor.set(-speed);
    }

}

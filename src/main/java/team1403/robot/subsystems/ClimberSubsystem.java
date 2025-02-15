package team1403.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.util.datalog.DataLog;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.robot.Constants;

public class ClimberSubsystem extends SubsystemBase{
    private SparkMax m_motor;
    private double speed;

    public ClimberSubsystem(){
        m_motor = new SparkMax(Constants.Climber.motorID, MotorType.kBrushless);
    }

    public void lift(double speed){
        m_motor.set(speed);
    }

    public void lower(double speed){
        m_motor.set(-speed); 
    }

}

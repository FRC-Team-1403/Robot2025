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
    private SparkMax m_motor;

    public ClimberSubsystem(){
        m_motor = new SparkMax(Constants.CanBus.ClimberMotor, MotorType.kBrushless);
        configMotors();
    }

    public void configMotors(){
        SparkMaxConfig rightConfig = new SparkMaxConfig();
        rightConfig 
            .idleMode(IdleMode.kBrake);
        rightConfig.smartCurrentLimit(50);
        m_motor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setMotorSpeed(double speed) {
        m_motor.set(speed);
    }

    public void stopMotors() {
        m_motor.set(0);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Climber/Right Speed", m_motor.get());
    }

}
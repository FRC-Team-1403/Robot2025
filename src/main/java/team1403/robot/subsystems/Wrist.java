package team1403.robot.subsystems;

import java.lang.constant.Constable;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.robot.Constants;

public class Wrist extends SubsystemBase{
    private SparkMax m_wristMotor;
    private final DutyCycleEncoder m_wristEncoder;
    private final ProfiledPIDController m_wristPid;
    private double m_wristAngleSetpoint;
    private MechanismLigament2d m_wristMech;

    public Wrist() {
        m_wristMotor = new SparkMax(0, MotorType.kBrushless);
        m_wristEncoder = new DutyCycleEncoder(Constants.RioPorts.kwristAbsoluteEncoder);
        

    configWristMotor();
    m_wristPid = new ProfiledPIDController(Constants.Wrist.KPWrist, Constants.Wrist.KIWrist, Constants.Wrist.KDWrist, new TrapezoidProfile.Constraints(370, 500));
    m_wristPid.reset(getWristAngle(), 0);
    m_wristPid.setIntegratorRange(-1,1);
    
    
    
        if(Constants.DEBUG_MODE) {
            //Constants.kDebugTab.addBoolean("Wrist is at Setpoint", isWristAtSetpoint());
            Constants.kDebugTab.add("Wrist PIDController", m_wristPid);
        }
        
        m_wristAngleSetpoint = Constants.Wrist.kIntakeSetpoint;
    }
    
        
    
    // private double calcWristSpeed() {
    //     double setpoint = m_wristAngleSetpoint;
    //     double speed = m_wristPid.calculate(getWristAngle(), setpoint);
    
    //     if (getWristAngle()) < Constants.Wrist.kWristLowerLimit)
    //     speed = MathUtil.clamp(speed, 0, 0.1);
    //     else if(getWristAngle() > Constants.Wrist.kWristUpperLimit)
    //     speed = MathUtil.clamp(speed, -0.1,0);
    //     return speed;    
        
    // }
    
    // private void configWristMotor() {
    //     m_wristMotor.setIdleMode(IdleMode.kBrake);
    // }
    
    public double getWristAngle() {
        return m_wristEncoder.getAbsolutePosition() * 360;
    }

    public boolean isWristAtSetpoint() {
        return Math.abs(getWristAngle() - m_wristAngleSetpoint) <= 5.0;
    }

    public void setWristSetpoint(double angle) {
        m_wristAngleSetpoint = MathUtil.clamp(angle, Constants.Wrist.kWristLowerLimit, Constants.Wrist.kWristUpperLimit);
    }
    

    private boolean isWristInSafeBounds() {
        return.getWristAngle() > 130 && getWristAngle() < 145;
    }

    private double calcWristSpeed() {
        double setpoint = m_wristAngleSetpoint;
        double speed = m_wristPid.calculate(getWristAngle(), setpoint);
        if(getWristAngle() < Constants.Wrist.kWristLowerLimit)
            speed = MathUtil.clamp(speed, 0, 0.1);
        else if(getWristAngle() > Constants.Wrist.kWristUpperLimit)
            speed = MathUtil.clamp(speed, -0.1, 0);

        return speed;
    }

    @Override
    public void periodic() {

        m_wristMech.setAngle(-getWristAngle() + 90);

        m_wristMotor.set(calcWristSpeed());

        DogLog.log("Wrist/Voltage", m_wristMotor.getBusVoltage());
        DogLog.log("Wrist/Current", m_wristMotor.getOutputCurrent());
        DogLog.log("Wrist/Temp", m_wristMotor.getMotorTemperature());
        DogLog.log("Wrist/Motor RPM", m_wristMotor.getEmbeddedEncoder().getVelocityValue());
        DogLog.log("Wrist/Speed", m_wristMotor.get());
        DogLog.log("Wrist/Angle", getWristAngle());
        DogLog.log("Wrist/Setpoint", m_wristAngleSetpoint);

    }


}




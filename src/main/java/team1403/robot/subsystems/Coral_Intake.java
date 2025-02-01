package team1403.robot.subsystems;

//import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Ultrasonic;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import team1403.robot.Constants;


public class Coral_Intake extends SubsystemBase {
    private SparkMax m_intakeMotor; 
    private Ultrasonic m_ultrasonic;

    //private final MotionMagicVelocityDutyCycle m_request = new MotionMagicVelocityDutyCycle(0);
    
    //private SysIdRoutine m_SysIdRoutine;
    
    public Coral_Intake() {
        m_intakeMotor = new SparkMax(Constants.Coral.motor, MotorType.kBrushless);
        m_ultrasonic = new Ultrasonic(Constants.Coral.pingChannel, Constants.Coral.echoChannel);
    }

    public void setMotorSpeed(double speed) {
        m_intakeMotor.set(speed);
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
}

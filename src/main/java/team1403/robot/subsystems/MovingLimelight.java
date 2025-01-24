package team1403.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DigitalInput;

import java.util.Optional;

import com.revrobotics.spark.SparkMax;
import team1403.robot.Constants;
import team1403.robot.vision.LimelightHelpers;
import team1403.robot.vision.LimelightWrapper;
import edu.wpi.first.math.util.Units;


public class MovingLimelight {
    private static SparkMax m_Motor;
    private static String m_name;

    public MovingLimelight() {
        m_Motor = new SparkMax(0, null);
    }

    public void startMotor(double speed) {
        m_Motor.set(speed);
    }

    public void stopMotor() {
        m_Motor.set(0);
    }

    public double currentSpeed() {
        return m_Motor.get();
    }
   
 

    public void periodic(){
       
    }
}
   
    


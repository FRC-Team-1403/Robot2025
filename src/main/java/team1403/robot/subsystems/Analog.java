package team1403.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Analog extends SubsystemBase {
    
    private AnalogPotentiometer m_Analog;
    
    public Analog() {

        m_Analog = new AnalogPotentiometer(0,180,0);      
        
    }

    public double getValue(){
        return m_Analog.get();
    }

    public void periodic(){
        Logger.recordOutput("Analog", getValue());
    }

}

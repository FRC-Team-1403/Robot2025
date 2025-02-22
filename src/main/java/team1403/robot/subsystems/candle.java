package team1403.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;

public class candle extends SubsystemBase {
    private CANdle m_candle;
    private Timer m_timer; 
    private int idx;
    private CoralIntakeSubsystem m_coralIntakeSubsystem;
    
    public candle() {
        m_candle = new CANdle(4);
        m_timer = new Timer(); 
        m_timer.start(); 
        
    }

    public void setLED() {

        m_candle.setLEDs(0,255,0, 0, 8, 26);
        m_candle.setLEDs(255, 0, 0, 0, 0, 8);

        // while (m_timer.get() % 0.0 == 0) {
        //     m_candle.setLEDs(100,100,100);
        // }
        // while (m_timer.get() % 0.5 == 0.0) {
        //     m_candle.setLEDs(0, 100, 30);
        // }       

    double brightness = 0;

    while (m_timer.get() <= 30) {
        if(!m_coralIntakeSubsystem.isCoralLoaded()) {
                    // If coral is not in, set the LED to red
            m_candle.setLEDs(242,2,34,0,0,8);
        } else {
                    // If coral is in, set the LED to green 
            m_candle.setLEDs(10,245,143,0,0,8);
        }

        if (m_timer.get() == 25) {
            for (int counter = 1; counter < 21; counter++) {
                m_candle.setLEDs(0,0,255,0,0,8);
                m_candle.configBrightnessScalar(brightness);

                if (brightness <= 1) {
                    brightness += 0.1;
                } else {
                    brightness -= 0.1;
                }
            }
        }

    }

        // if (m_timer.get() >= 0.3) { 
           
        //     switch (idx % 8) {
        //         case 0:
        //             m_candle.setLEDs(0, 25, 0, 0, 0, 4); 
        //             m_candle.setLEDs(25, 25, 0, 0, 4, 4); 
        //             break;

        //         case 1:
        //             m_candle.setLEDs(0, 25, 0, 0, 1, 4); 
        //             m_candle.setLEDs(25, 25, 0, 0, 5, 3); 
        //             m_candle.setLEDs(25, 25, 0, 0, 0, 1); 
        //             break;

        //         case 2:
        //             m_candle.setLEDs(0, 25, 0, 0, 2, 4); 
        //             m_candle.setLEDs(25, 25, 0, 0, 6, 2); 
        //             m_candle.setLEDs(25, 25, 0, 0, 0, 2); 
        //             break;

        //         case 3:
        //             m_candle.setLEDs(0, 25, 0, 0, 3, 4); 
        //             m_candle.setLEDs(25, 25, 0, 0, 7, 1); 
        //             m_candle.setLEDs(25, 25, 0, 0, 0, 3); 
        //             break;

        //         case 4:
        //             m_candle.setLEDs(0, 25, 0, 0, 4, 4); 
        //             m_candle.setLEDs(25, 25, 0, 0, 0, 4); 

        //         case 5:
        //             m_candle.setLEDs(0, 25, 0, 0, 5, 3); 
        //             m_candle.setLEDs(0, 25, 0, 0, 0, 1); 
        //             m_candle.setLEDs(25, 25, 0, 0, 1, 4); 
        //             break;

        //         case 6:
        //             m_candle.setLEDs(0, 25, 0, 0, 2, 4); 
        //             m_candle.setLEDs(25, 25, 0, 0, 6, 2); 
        //             m_candle.setLEDs(25, 25, 0, 0, 0, 2); 
        //             break;

        //         case 7:
        //             m_candle.setLEDs(0, 25, 0, 0, 3, 4);
        //             m_candle.setLEDs(25, 25, 0, 0, 7, 1);
        //             m_candle.setLEDs(25, 25, 0, 0, 0, 3);
        //             break;
        //     }

        //     idx++;
        //     if (idx >= 8) {
        //         idx = 0; 
        //     }
        //     System.out.println((idx % 8)); 
        //     m_timer.reset();
        // }

       
    }
}

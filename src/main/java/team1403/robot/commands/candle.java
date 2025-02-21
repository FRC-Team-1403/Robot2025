
package team1403.robot.commands;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj.Timer;


public class candle {
    public CANdle m_candle = new CANdle(4);
    Timer m_Timer = new Timer();

    public void setCandle() {
        m_candle.setLEDs(255, 255, 255);
    }
}
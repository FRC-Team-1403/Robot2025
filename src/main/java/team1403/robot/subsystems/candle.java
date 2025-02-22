package team1403.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;

public class candle extends SubsystemBase {
    private CANdle m_candle;
    
    public candle() {
        m_candle = new CANdle(4);
    }

    public void setLED(int r, int g, int b) {
        m_candle.setLEDs(r, g, b);
    }
}

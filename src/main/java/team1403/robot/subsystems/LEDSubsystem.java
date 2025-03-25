package team1403.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.robot.Constants;

public class LEDSubsystem extends SubsystemBase {

    private final CANdle m_candle;

    public enum Color {
        Green,
        Red,
        Off,
        Blue,
        Grey,
        Pink,
        White,
        Yellow,
        Brown,
        Orange,
        Purple
    }

    public LEDSubsystem() {
        m_candle = new CANdle(Constants.CanBus.kCandleID);
        CANdleConfiguration config = new CANdleConfiguration();
        m_candle.configAllSettings(config);
    }

    public void setLEDcolor(Color color) {
        switch(color) {
            case Green:
                m_candle.setLEDs(0, 255, 0);
                break;
            case Red:
                m_candle.setLEDs(255, 0, 0);
                break;
            case Off:
                m_candle.setLEDs(0, 0, 0);
                break;
            case Blue:
                m_candle.setLEDs(0, 0, 255);
                break;
            case Grey:
                m_candle.setLEDs(128, 128, 128);
                break;
            case Pink:
                m_candle.setLEDs(255, 182, 193);
                break;
            case White:
                m_candle.setLEDs(255, 255, 255);
                break;
            case Yellow:
                m_candle.setLEDs(255, 255, 0);
                break;
            case Brown:
                m_candle.setLEDs(139, 69, 19);
                break;
            case Orange:
                m_candle.setLEDs(255, 165, 0);
                break;
            case Purple:
                m_candle.setLEDs(128, 0, 128);
                break;
        }
    }
    
    public void setLEDcolor(Color color, int startIndex, int count) {
        switch(color) {
            case Green:
                m_candle.setLEDs(0, 255, 0, 0, startIndex, count); // RGB for Green
                break;
            case Red:
                m_candle.setLEDs(255, 0, 0, 0, startIndex, count); // RGB for Red
                break;
            case Off:
                m_candle.setLEDs(0, 0, 0, 0, startIndex, count); // RGB for Black (off)
                break;
            case Blue:
                m_candle.setLEDs(0, 0, 255, 0, startIndex, count); // RGB for Blue
                break;
            case Grey:
                m_candle.setLEDs(128, 128, 128, 0, startIndex, count); // RGB for Grey
                break;
            case Pink:
                m_candle.setLEDs(255, 182, 193, 0, startIndex, count); // RGB for Pink
                break;
            case White:
                m_candle.setLEDs(255, 255, 255, 0, startIndex, count); // RGB for White
                break;
            case Yellow:
                m_candle.setLEDs(255, 255, 0, 0, startIndex, count); // RGB for Yellow
                break;
            case Brown:
                m_candle.setLEDs(139, 69, 19, 0, startIndex, count); // RGB for Brown
                break;
            case Orange:
                m_candle.setLEDs(255, 165, 0, 0, startIndex, count); // RGB for Orange
                break;
            case Purple:
                m_candle.setLEDs(128, 0, 128, 0, startIndex, count); // RGB for Purple
                break;
        }
    }
    
    
    
    @Override
    public void periodic() {}
}

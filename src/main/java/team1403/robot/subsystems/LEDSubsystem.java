package team1403.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.robot.Constants;

public class LEDSubsystem extends SubsystemBase {

    private final CANdle m_candle;

    enum LEDState {
        OFF,
        DEFAULT
    }

    private LEDState m_state;

    public LEDSubsystem() {
        m_candle = new CANdle(Constants.CanBus.candleID);
        CANdleConfiguration config = new CANdleConfiguration();
        m_candle.configAllSettings(config);

        m_state = LEDState.DEFAULT;
    }

    public void setLedState(LEDState state)
    {
        m_state = state;
    }
    
    @Override
    public void periodic() {
        switch (m_state)
        {
            case DEFAULT:
                //TODO: what default pattern do we want?
                break;
            case OFF:
            default:
                m_candle.setLEDs(0, 0, 0);
                break;
        }
    }
}

package team1403.robot.subsystems;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.robot.Constants;

public class LEDSubsystem extends SubsystemBase {
    private final CANdle m_candle;
    private final int m_ledCount = Constants.LED.kLedCount;
    private final double m_animSpeed = Constants.LED.speed;
    
    public enum LEDConfig {
        ; //It just works don't question it
        public enum Style {
            Solid,
            Rainbow,
            Strobe,
            Upwards,
            Downwards
        }
        
        public enum Color {
            Green(0, 255, 0),
            Red(255, 0, 0),
            Blue(0, 0, 255),
            Grey(128, 128, 128),
            Pink(255, 182, 193),
            White(255, 255, 255),
            Yellow(255, 255, 0),
            Brown(139, 69, 19),
            Orange(255, 165, 0),
            Purple(128, 0, 128),
            Off(0,0,0);
    
            private final int red;
            private final int green;
            private final int blue;
    
            Color(int red, int green, int blue) {
                this.red = red;
                this.green = green;
                this.blue = blue;
            }
            
            public int getRed() {
                return red;
            }
            
            public int getGreen() {
                return green;
            }
            
            public int getBlue() {
                return blue;
            }
        }
    }

    public LEDSubsystem() {
        m_candle = new CANdle(Constants.CanBus.kCandleID);
        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB;
        config.brightnessScalar = 1;
        m_candle.configAllSettings(config);
        m_candle.setLEDs(0, 0, 0);
    }

    public void setLEDcolor(LEDConfig.Color color) {
        setLEDcolor(LEDConfig.Style.Solid, color);
    }


    public void setLEDcolor(LEDConfig.Style style, LEDConfig.Color primary) {
        if (primary == null) primary = LEDConfig.Color.Off;
        
        int P_red = primary.getRed();
        int P_green = primary.getGreen();
        int P_blue = primary.getBlue();

        switch(style) {
            case Solid:
                m_candle.setLEDs(P_red, P_green, P_blue);
                break;
                
            case Rainbow:
                m_candle.animate(new RainbowAnimation(1.0, m_animSpeed, m_ledCount));
                break;
                
            case Strobe:
                m_candle.animate(new LarsonAnimation(P_red, P_green, P_blue, 0,
                                                    m_animSpeed, 
                                                    m_ledCount, 
                                                    LarsonAnimation.BounceMode.Center, 7));
                break;
                
            case Upwards:
                m_candle.animate(new ColorFlowAnimation(P_red, P_green, P_blue, 0,
                                                        m_animSpeed,
                                                        m_ledCount, 
                                                        ColorFlowAnimation.Direction.Forward));
                break;
                
            case Downwards:
                m_candle.animate(new ColorFlowAnimation(P_red, P_green, P_blue, 0,
                                                        m_animSpeed,
                                                        m_ledCount, 
                                                        ColorFlowAnimation.Direction.Backward));
                break;
                
            default:
                m_candle.setLEDs(0, 0, 0);
                break;
        }
    }
    
    public void setBrightness(double brightness) {
        m_candle.configBrightnessScalar(brightness);
    }
    
    @Override
    public void periodic() {}
}

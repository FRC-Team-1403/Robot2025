package team1403.robot.subsystems;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.lib.util.StripedAnimation;
import team1403.robot.Constants;

public class LEDSubsystem extends SubsystemBase {
    private final CANdle m_candle;
    private final int m_ledCount = Constants.LED.kLedCount;
    private final double m_animSpeed = Constants.LED.speed;
    private StripedAnimation m_stripedAnimation;
    
    public enum LEDConfig {
        ; //It just works don't question it
        public enum Style {
            Solid,
            Rainbow,
            Striped,
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
        m_stripedAnimation = null;
        m_candle = new CANdle(Constants.CanBus.kCandleID);
        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB; // Or GRB depending on your LED strip
        config.brightnessScalar = 0.8; // 80% brightness by default
        m_candle.configAllSettings(config);
        m_candle.setLEDs(0, 0, 0); // Start with LEDs off
    }

    public void setLEDcolor(LEDConfig.Style style) {
        setLEDcolor(style, LEDConfig.Color.Red, null); // Default color if none provided
    }

    public void setLEDcolor(LEDConfig.Style style, LEDConfig.Color color) {
        setLEDcolor(style, color, null);
    }

    public void setLEDcolor(LEDConfig.Style style, LEDConfig.Color primary, LEDConfig.Color secondary) {
        if (primary == null) primary = LEDConfig.Color.Off;
        if (secondary == null) secondary = LEDConfig.Color.Off;
        
        // Fixed color channel assignments (you had green/blue swapped)
        int P_red = primary.getRed();
        int P_green = primary.getGreen();
        int P_blue = primary.getBlue();

        int S_red = secondary.getRed();
        int S_green = secondary.getGreen();
        int S_blue = secondary.getBlue();

        switch(style) {
            case Solid:
                m_candle.setLEDs(P_red, P_green, P_blue);
                break;
                
            case Rainbow:
                m_candle.animate(new RainbowAnimation(1.0, m_animSpeed, m_ledCount));
                break;
                
            case Striped:
                m_candle.setLEDs(0, 0, 0);
                m_stripedAnimation = new StripedAnimation(m_candle, m_ledCount,
                                                          P_red, P_green, P_blue,
                                                          S_red, S_green, S_blue,
                                                          Constants.LED.speed);
                m_stripedAnimation.start();
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
                m_candle.setLEDs(0, 0, 0); // Turn off if unknown style
                break;
        }
    }
    
    public void setBrightness(double brightness) {
        m_candle.configBrightnessScalar(brightness);
    }
    
    @Override
    public void periodic() {
        // Animation updates happen automatically
    }
}

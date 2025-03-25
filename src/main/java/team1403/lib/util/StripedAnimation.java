package team1403.lib.util;

import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class StripedAnimation{
    private final CANdle m_candle;
    private final int m_ledCount;
    private final Color8Bit m_primary;
    private final Color8Bit m_secondary;
    private final double m_speed;
    private int m_offset = 0;
    private boolean m_isRunning = false;
    private Thread m_animationThread;

    public StripedAnimation(CANdle candle, int ledCount,
                          int primaryRed, int primaryGreen, int primaryBlue,
                          int secondaryRed, int secondaryGreen, int secondaryBlue,
                          double speed) {
        this.m_candle = candle;
        this.m_ledCount = ledCount;
        this.m_primary = new Color8Bit(primaryRed, primaryGreen, primaryBlue);
        this.m_secondary = new Color8Bit(secondaryRed, secondaryGreen, secondaryBlue);
        this.m_speed = speed;
    }

    public void start() {
        if (m_isRunning) return;
        
        m_isRunning = true;
        m_animationThread = new Thread(() -> {
            while (m_isRunning) {
                // Update LED pattern
                for (int i = 0; i < m_ledCount; i++) {
                    int patternPos = (i + m_offset) % 4;
                    Color8Bit color = (patternPos < 2) ? m_primary : m_secondary;
                    m_candle.setLEDs(color.red, color.green, color.blue, 0, i, 1);
                }
                
                // Update offset
                m_offset = (m_offset + 1) % 4;
                
                // Control speed
                try {
                    Thread.sleep((long)(1000 / (30 * m_speed))); // 30 FPS base
                } catch (InterruptedException e) {
                    break;
                }
            }
        });
        m_animationThread.start();
    }

    public void stop() {
        m_isRunning = false;
        if (m_animationThread != null) {
            m_animationThread.interrupt();
        }
        // Turn off LEDs
        m_candle.setLEDs(0, 0, 0);
    }
}
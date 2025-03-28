package team1403.lib.util;

import edu.wpi.first.wpilibj.Timer;

/**
 * This class contains a boolean value and a timer. It can set its boolean value and return whether the timer is within
 * a set timeout. This returns true if the stored value is true and the timeout has expired.
 * Thanks team 254
 */
public class TimeDelayedBoolean {
    private Timer t = new Timer();
    private boolean m_old = false;

    public boolean update(boolean value, double timeout) {
        if (!m_old && value) {
            t.restart();
        }
        m_old = value;
        return value && t.get() >= timeout;
    }

    public void reset() {
        t.restart();
    }
}
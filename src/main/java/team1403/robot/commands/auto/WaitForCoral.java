package team1403.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.subsystems.Blackbox;

public class WaitForCoral extends Command {

    private final Timer m_timer;
    private final double m_timeout;

    public WaitForCoral(double timeout)
    {
        m_timer = new Timer();
        m_timeout = timeout;
    }

    @Override
    public void initialize() {
        m_timer.restart();
    }

    @Override
    public boolean isFinished() {
        return Blackbox.isCoralLoaded() || m_timer.hasElapsed(m_timeout);
    }
    
}

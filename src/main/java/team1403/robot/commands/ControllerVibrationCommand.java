package team1403.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;

public class ControllerVibrationCommand extends Command {

    private XboxController m_controller;
    private double m_length;
    private double m_strength;
    private Timer m_timer;

    public ControllerVibrationCommand(XboxController controller, double length, double strength) {
        m_controller = controller;
        m_length = length;
        m_strength = strength;

        m_timer = new Timer();
    }

    @Override
    public void initialize() {
        m_timer.restart();
        m_controller.setRumble(RumbleType.kBothRumble, m_strength);
    }

    @Override
    public void end(boolean interrupt) {
        m_controller.setRumble(RumbleType.kBothRumble, 0);
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(m_length);
    }
}

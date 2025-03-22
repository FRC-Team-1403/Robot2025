package team1403.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;

public class ControllerVibrationCommand extends Command {

    //Variable for local copy of the controller to vibrate
    private XboxController m_controller;
    //Variable for local copy of the length in seconds
    private double m_length;
    //Variable for local copy of the strength of vibration [0,1]
    private double m_strength;
    //Variable for local copy of the timer object
    private Timer m_timer;

    /**
     * Vibrates the controller
     * 
     * @param controller the controller to operate on
     * @param length how long to vibrate teh controller (seconds)
     * @param strength how strong the vibration will be
     */
    public ControllerVibrationCommand(XboxController controller, double length, double strength) {
        m_controller = controller;
        m_length = length;
        m_strength = strength;

        m_timer = new Timer();
    }

    @Override
    public void initialize() {
        //Restarts the timer
        m_timer.restart();
        //Sets the controller rumble
        m_controller.setRumble(RumbleType.kBothRumble, m_strength);
    }

    @Override
    public void end(boolean interrupt) {
        //Stops the controller rumble AKA setting the rumble to 0
        m_controller.setRumble(RumbleType.kBothRumble, 0);
    }

    @Override
    public boolean isFinished() {
        //Ends the command when the time in the timer = the time specified
        return m_timer.hasElapsed(m_length);
    }
}
